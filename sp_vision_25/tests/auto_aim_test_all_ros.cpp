#include <chrono>

#include <fmt/core.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "io/camera.hpp"
#include "io/ros2/gimbal_ros.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                   | 输出命令行参数说明 }"
  "{@config-path   | configs/demo.yaml | yaml配置文件路径}";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto config = YAML::LoadFile(config_path);
  const auto solver_config_path = config["calibration_path"]
    ? config["calibration_path"].as<std::string>()
    : config_path;

  tools::Exiter exiter;

  rclcpp::init(argc, argv);

  io::Camera camera(config_path);
  auto_aim::YOLO yolo(config_path);
  auto_aim::Solver solver(solver_config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);
  auto gimbal = std::make_shared<io::GimbalROS>();
  gimbal->start_spin();

  cv::Mat img;

  double fps = 0.0;
  auto last_fps_time = std::chrono::steady_clock::now();
  int frame_count = 0;
  constexpr double fps_update_interval = 1.0;

  bool last_frame_detected_target = false;
  double last_sent_yaw = 0.0;
  double last_sent_pitch = 0.0;

  while (!exiter.exit()) {
    auto frame_start = std::chrono::steady_clock::now();

    camera.read(img, frame_start);
    if (img.empty()) {
      break;
    }

    constexpr auto IMU_DELAY = 2ms;
    Eigen::Quaterniond gimbal_q = gimbal->q(frame_start - IMU_DELAY);
    solver.set_R_gimbal2world(gimbal_q);
    Eigen::Vector3d ypr = tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0);

    auto armors = yolo.detect(img, 0);
    auto targets = tracker.track(armors, frame_start);
    auto command = aimer.aim(targets, frame_start, 19.6, true);
    command.shoot = shooter.shoot(command, aimer, targets, ypr);

    bool detected_target = !armors.empty();

    // ===== 新增：准备广播目标坐标 =====
    bool need_publish_target_xyz = false;
    Eigen::Vector3d publish_xyz_camera = Eigen::Vector3d::Zero();
    Eigen::Vector3d publish_xyz_gimbal = Eigen::Vector3d::Zero();

    if (detected_target) {
      const auto & best_armor = armors.front();
      publish_xyz_gimbal = best_armor.xyz_in_gimbal;
      publish_xyz_camera =
        solver.R_camera2gimbal().transpose() *
        (publish_xyz_gimbal - solver.t_camera2gimbal());
      need_publish_target_xyz = true;
    }

    if (detected_target) {
      gimbal->send(command.control, command.shoot, command.yaw, 0, 0, command.pitch, 0, 0);

      // ===== 新增：发送后同步广播目标在相机系/云台系下的位置 =====
      if (need_publish_target_xyz) {
        gimbal->publish_target_xyz(
          publish_xyz_camera,
          publish_xyz_gimbal,
          gimbal->now());
      }

      last_sent_yaw = command.yaw;
      last_sent_pitch = command.pitch;
      last_frame_detected_target = true;
    } else if (last_frame_detected_target) {
      gimbal->send(true, false, last_sent_yaw, 0, 0, last_sent_pitch, 0, 0);

      command.yaw = last_sent_yaw;
      command.pitch = last_sent_pitch;
      command.shoot = false;
      command.control = true;

      last_frame_detected_target = false;
    }

    frame_count++;
    auto now = std::chrono::steady_clock::now();
    double elapsed = tools::delta_time(now, last_fps_time);

    if (elapsed >= fps_update_interval) {
      fps = frame_count / elapsed;
      frame_count = 0;
      last_fps_time = now;

      tools::logger()->info("FPS: {:.1f}, Tracker State: {}", fps, tracker.state());
    }

    // 绿色：识别结果；青色：跟踪器预测；红色/蓝色：有效/无效瞄准点。
    for (const auto & armor : armors) {
      if (armor.points.size() == 4) {
        tools::draw_points(img, armor.points, {0, 255, 0});
      }
      cv::circle(img, armor.center, 4, {0, 255, 0}, -1);
      tools::draw_text(
        img, fmt::format("{} {:.2f}", auto_aim::ARMOR_NAMES[armor.name], armor.confidence),
        {static_cast<int>(armor.center.x), static_cast<int>(armor.center.y) - 12},
        {0, 255, 0}, 0.6, 2);
    }

    if (!targets.empty()) {
      const auto & target = targets.front();
      for (const auto & xyza : target.armor_xyza_list()) {
        tools::draw_points(
          img, solver.reproject_armor(xyza.head<3>(), xyza[3], target.armor_type, target.name),
          {255, 255, 0});
      }
      const auto & aim_point = aimer.debug_aim_point;
      tools::draw_points(
        img,
        solver.reproject_armor(
          aim_point.xyza.head<3>(), aim_point.xyza[3], target.armor_type, target.name),
        aim_point.valid ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0));
    }

    tools::draw_text(
      img,
      fmt::format("FPS {:.1f} | {} | armors {} | targets {}", fps, tracker.state(),
                  armors.size(), targets.size()),
      {10, 30}, {255, 255, 255}, 0.7, 2);
    tools::draw_text(
      img,
      fmt::format("control {} | yaw {:.1f} | pitch {:.1f} | shoot {}",
                  command.control, command.yaw * 57.3, command.pitch * 57.3, command.shoot),
      {10, 60}, {255, 255, 255}, 0.7, 2);

    cv::Mat display;
    cv::resize(img, display, {}, 0.5, 0.5);
    cv::imshow("Single Camera Auto Aim", display);
    const int key = cv::waitKey(1);
    if (key == 'q' || key == 27) break;
  }

  cv::destroyWindow("Single Camera Auto Aim");
  rclcpp::shutdown();
  return 0;
}
