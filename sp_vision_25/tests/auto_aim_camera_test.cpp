#include <fmt/core.h>
#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"              // 使用项目封装的工业相机接口
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

int main(int argc, char *argv[])
{
  tools::Exiter exiter;

  // 配置文件路径 - 指定工业相机和YOLO模型参数
  std::string config_path = "configs/sentry.yaml";

  // 初始化模块
  io::Camera camera(config_path);   // ← 用工业相机
  auto_aim::YOLO yolo(config_path);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);

  cv::Mat frame;
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

  int frame_count = 0;
  double last_t = -1;
  io::Command last_command{false, false, 0, 0};

  // 固定四元数（因为没有IMU）
  Eigen::Quaterniond fixed_quaternion(1, 0, 0, 0);

  while (!exiter.exit()) {
    std::chrono::steady_clock::time_point t;
    camera.read(frame, t);   // ← 从工业相机读取一帧

    if (frame.empty()) {
      tools::logger()->error("工业相机读取失败");
      break;
    }

    double timestamp = std::chrono::duration<double>(t - t0).count();

    // 没有IMU → 固定姿态
    solver.set_R_gimbal2world({fixed_quaternion.w(), fixed_quaternion.x(),
                              fixed_quaternion.y(), fixed_quaternion.z()});

    // YOLO检测
    auto armors = yolo.detect(frame, frame_count);

    // 目标跟踪
    auto targets = tracker.track(armors, t);

    // 瞄准解算
    auto command = aimer.aim(targets, t, 27, false);

    // 简单射击逻辑
    if (last_command.control && command.control && !targets.empty() &&
        aimer.debug_aim_point.valid &&
        std::abs(command.yaw - last_command.yaw) * 57.3 < 2) {
      command.shoot = true;
    }
    if (command.control) last_command = command;

    // 绘制检测结果
    for (const auto& armor : armors) {
      // points 是原图坐标，ROI 模式下也可直接用于绘制。
      if (armor.points.size() == 4) {
        for (size_t i = 0; i < armor.points.size(); ++i) {
          cv::line(frame, armor.points[i], armor.points[(i + 1) % armor.points.size()],
                   cv::Scalar(0, 255, 0), 2);
        }
      }
      cv::circle(frame,
        cv::Point2f(armor.center_norm.x * frame.cols,
                    armor.center_norm.y * frame.rows),
        5, cv::Scalar(0, 0, 255), -1);

      std::string label = fmt::format("Conf: {:.2f}", armor.confidence);
      cv::putText(frame, label,
                  cv::Point2f(armor.center_norm.x * frame.cols,
                              armor.center_norm.y * frame.rows - 10),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  cv::Scalar(255, 255, 0), 1);
    }

    // 显示命令
    tools::draw_text(
      frame,
      fmt::format("Tracker: {} | Targets: {} | Control: {} | Yaw {:.2f}° | Pitch {:.2f}° | Shoot: {}",
        tracker.state(), targets.size(), command.control ? "YES" : "NO",
        command.yaw * 57.3, command.pitch * 57.3,
        command.shoot ? "YES" : "NO"),
      {10, 30}, {0, 255, 0});

    // 显示图像
    cv::imshow("Industrial Camera Auto-Aim", frame);

    int key = cv::waitKey(1);
    if (key == 'q' || key == 27) break;
    else if (key == ' ') cv::waitKey(0);

    frame_count++;
  }

  cv::destroyAllWindows();
  return 0;
}
