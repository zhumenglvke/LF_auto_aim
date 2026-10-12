#include <fmt/core.h>

#include <chrono>
#include <memory>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <yaml-cpp/yaml.h>

#include "io/camera.hpp"
#include "io/ros2/gimbal_ros.hpp"

#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/omniperception/decider.hpp"

#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/viewer_tool.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

// ==========================================
// 多线程取帧模型
// ==========================================
struct FrameData {
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
};

class CameraStream {
public:
    CameraStream(io::SNCamera* cam) : camera_(cam), running_(true) {
        thread_ = std::thread(&CameraStream::grab_loop, this);
    }

    ~CameraStream() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
    }

    bool get_latest(cv::Mat& out_img, std::chrono::steady_clock::time_point& out_t) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (latest_frame_.img.empty()) return false;
        
        // 【优化】使用 copyTo 替代 clone()。只要 out_img 在外部不被销毁，
        // copyTo 会自动复用内存，避免每帧分配数MB内存的巨大开销！
        latest_frame_.img.copyTo(out_img); 
        out_t = latest_frame_.timestamp;
        return true;
    }

private:
    void grab_loop() {
        while (running_) {
            cv::Mat img;
            auto t = std::chrono::steady_clock::now();
            camera_->read(img, t); // 持续抓取，清空底层缓存
            
            if (!img.empty()) {
                std::lock_guard<std::mutex> lock(mutex_);
                latest_frame_.img = img;
                latest_frame_.timestamp = t;
            }
        }
    }

    io::SNCamera* camera_;
    std::atomic<bool> running_;
    std::thread thread_;
    std::mutex mutex_;
    FrameData latest_frame_;
};
// ==========================================

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/demo.yaml      | 位置参数，yaml配置文件路径}";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  auto config_path = cli.get<std::string>(0);
  const bool use_windows = YAML::LoadFile(config_path)["use_windows"].as<bool>(false);
  std::unique_ptr<tools::ViewerTool> viewer;
  if (use_windows) {
    viewer = std::make_unique<tools::ViewerTool>("Dual Camera View", 0.5);
  }
  rclcpp::init(argc, argv);

  tools::Exiter exiter;
  auto gimbal = std::make_shared<io::GimbalROS>();
  gimbal->start_spin();

  // 初始化相机
  io::SNCamera camera(config_path);
  std::this_thread::sleep_for(std::chrono::seconds(2));
  io::SNCamera back_camera("configs/cam2.yaml");

  // 将相机包装进多线程流中
  CameraStream front_stream(&camera);
  CameraStream back_stream(&back_camera);

  // 初始化自瞄与决策模块
  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);
  auto tracker = std::make_unique<auto_aim::Tracker>(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);
  omniperception::Decider decider(config_path);

  // 关键参数
  constexpr double BULLET_SPEED = 19.6;
  constexpr auto IMU_DELAY = 2ms;

  // 状态机参数
  bool waiting_front_lock = false;
  std::chrono::steady_clock::time_point waiting_front_lock_until = std::chrono::steady_clock::now();
  constexpr int BACK_TO_FRONT_HANDOFF_MS = 800;

  constexpr int BACK_CAM_COOLDOWN_MS = 2000; 
  std::chrono::steady_clock::time_point back_cam_cooldown_until = std::chrono::steady_clock::now();

  constexpr int CLEAR_FRONT_ARMOR_FRAMES_AFTER_BACK = 3;
  int clear_front_armor_frames = 0;

  bool last_frame_front_detected_target = false;
  double last_sent_yaw = 0.0;
  double last_sent_pitch = 0.0;
  io::Command last_command{false, false, 0, 0};

  auto clear_front_target_cache = [&]() {
    last_frame_front_detected_target = false;
    last_sent_yaw = 0.0;
    last_sent_pitch = 0.0;
    tracker = std::make_unique<auto_aim::Tracker>(config_path, solver);
    clear_front_armor_frames = CLEAR_FRONT_ARMOR_FRAMES_AFTER_BACK;
  };

  double fps = 0.0;
  auto last_fps_time = std::chrono::steady_clock::now();
  int frame_count = 0;

  // 【优化】将图片变量提到循环外部，全程复用这两块内存，榨干性能！
  cv::Mat front_img;
  cv::Mat back_img;

  while (!exiter.exit()) {
    std::chrono::steady_clock::time_point frame_start;

    // 从前摄线程流中获取最新帧，不阻塞
    if (!front_stream.get_latest(front_img, frame_start)) {
        std::this_thread::sleep_for(1ms);
        continue;
    }

    Eigen::Quaterniond gimbal_q = gimbal->q(frame_start - IMU_DELAY);
    solver.set_R_gimbal2world(gimbal_q);
    Eigen::Vector3d gimbal_pos = tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0);

    // =========================
    // 前摄识别
    // =========================
    auto armors = yolo.detect(front_img, 0);

    if (clear_front_armor_frames > 0) {
      armors.clear();
      clear_front_armor_frames--;
    }

    decider.armor_filter(armors);
    decider.set_priority(armors);

    auto targets = tracker->track(armors, frame_start);
    io::Command command{false, false, 0, 0};
    auto now = std::chrono::steady_clock::now();

    const bool front_has_detection = !armors.empty();
    const bool front_tracker_lost = tracker->state() == "lost";

    bool command_from_front = false;
    bool command_from_back = false;

    // ===== 新增：只保存“前摄当前画面”给出的 xyz =====
    bool need_publish_front_xyz = false;
    Eigen::Vector3d publish_xyz_camera = Eigen::Vector3d::Zero();
    Eigen::Vector3d publish_xyz_gimbal = Eigen::Vector3d::Zero();

    // 注意：必须同时满足
    // 1. 前摄 tracker 没有 lost，也就是前摄锁定
    // 2. 当前这一帧前摄 armors 不为空
    // 这样可以避免 tracker 靠历史状态维持时继续发旧 xyz
    if (!front_tracker_lost && front_has_detection) {
      const auto & best_armor = armors.front();

      publish_xyz_gimbal = best_armor.xyz_in_gimbal;
      publish_xyz_camera =
        solver.R_camera2gimbal().transpose() *
        (publish_xyz_gimbal - solver.t_camera2gimbal());

      need_publish_front_xyz = true;
    }

    // =========================
    // 自瞄与决策控制逻辑
    // =========================
    if (!front_tracker_lost) {
      // 前摄已锁定
      waiting_front_lock = false;
      
      // 只要前摄在锁定目标，持续刷新后摄冷却时间
      back_cam_cooldown_until = now + std::chrono::milliseconds(BACK_CAM_COOLDOWN_MS);

      command = aimer.aim(targets, frame_start, BULLET_SPEED, true);
      command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);
      command_from_front = true;
    } else {
      // 前摄彻底丢失
      if (waiting_front_lock && now < waiting_front_lock_until) {
        // 等待前摄接管中，保持沉默
        command = io::Command{false, false, 0, 0};
      } else if (now > back_cam_cooldown_until) {
        // 允许后摄介入
        std::chrono::steady_clock::time_point back_t;
        
        if (back_stream.get_latest(back_img, back_t)) {
             command = decider.decide(yolo, gimbal_pos, back_img, back_t, "back");
             
             if (command.control) {
                 command_from_back = true;
                 waiting_front_lock = true;
                 waiting_front_lock_until = now + std::chrono::milliseconds(BACK_TO_FRONT_HANDOFF_MS);
                 
                 back_cam_cooldown_until = now + std::chrono::milliseconds(BACK_CAM_COOLDOWN_MS);

                 armors.clear();
                 targets.clear();
                 clear_front_target_cache();
             }
        }
      }
    }

    // =========================
    // 发送逻辑 (完全保持你的电控安全逻辑)
    // =========================
    if (command.control) {
      gimbal->send(command.control, command.shoot, command.yaw, 0, 0, command.pitch, 0, 0);
      last_command = command;
      // 后摄 command、等待接管、丢目标保持角度，都不会进这里
      if (command_from_front && need_publish_front_xyz) {
        gimbal->publish_target_xyz(
          publish_xyz_camera,
          publish_xyz_gimbal,
          gimbal->now());
      }

      if (command_from_front && front_has_detection) {
        last_sent_yaw = command.yaw;
        last_sent_pitch = command.pitch;
        last_frame_front_detected_target = true;
      } else if (command_from_back) {
        last_frame_front_detected_target = false;
        last_sent_yaw = 0.0;
        last_sent_pitch = 0.0;
      }
    } else if (last_frame_front_detected_target && !waiting_front_lock && clear_front_armor_frames == 0) {
      // 刚丢目标的第一帧，保持上一帧角度
      gimbal->send(true, false, last_sent_yaw, 0, 0, last_sent_pitch, 0, 0);
      last_frame_front_detected_target = false;
    } 

    // =========================
    // FPS 监控 (极简版)
    // =========================
    frame_count++;
    auto loop_end = std::chrono::steady_clock::now();
    double elapsed = tools::delta_time(loop_end, last_fps_time);
    
    if (elapsed >= 1.0) {
      fps = frame_count / elapsed;
      frame_count = 0;
      last_fps_time = loop_end;

      // 【优化】使用 \r 实现原位覆盖刷新，不再刷屏阻塞终端
      fmt::print("\r[Vision] FPS: {:.1f}", fps);
      fflush(stdout); 
    }

    if (viewer) {
      std::chrono::steady_clock::time_point back_frame_time;
      back_stream.get_latest(back_img, back_frame_time);
      if (!viewer->update({front_img, back_img})) break;
    }
  }

  fmt::print("\n");
  rclcpp::shutdown();
  return 0;
}
