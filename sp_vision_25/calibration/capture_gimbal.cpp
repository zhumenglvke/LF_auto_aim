// #include <fmt/core.h>
// #include <filesystem>
// #include <fstream>
// #include <opencv2/opencv.hpp>

// #include "io/camera.hpp"
// #include "io/gimbal/gimbal.hpp"  // 改为使用Gimbal类
// #include "tools/img_tools.hpp"
// #include "tools/logger.hpp"
// #include "tools/math_tools.hpp"

// const std::string keys =
//   "{help h usage ?  |                          | 输出命令行参数说明}"
//   "{@config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
//   "{output-folder o |      assets/img_with_q   | 输出文件夹路径   }";

// void write_q(const std::string q_path, const Eigen::Quaterniond & q)
// {
//   std::ofstream q_file(q_path);
//   Eigen::Vector4d xyzw = q.coeffs();
//   // 输出顺序为wxyz
//   q_file << fmt::format("{} {} {} {}", xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
//   q_file.close();
// }

// void capture_loop(
//   const std::string & config_path, const std::string & output_folder)
// {
//   io::Gimbal gimbal(config_path);  // 使用Gimbal类替代CBoard
//   io::Camera camera(config_path);
//   cv::Mat img;
//   std::chrono::steady_clock::time_point timestamp;

//   int count = 0;
//   while (true) {
//     camera.read(img, timestamp);
//     Eigen::Quaterniond q = gimbal.q(timestamp);  // 通过Gimbal获取四元数

//     // 在图像上显示欧拉角，用来判断imuabs系的xyz正方向，同时判断imu是否存在零漂
//     auto img_with_ypr = img.clone();
//     Eigen::Vector3d zyx = tools::eulers(q, 2, 1, 0) * 57.3;  // degree
//     tools::draw_text(img_with_ypr, fmt::format("Z {:.2f}", zyx[0]), {40, 40}, {0, 0, 255});
//     tools::draw_text(img_with_ypr, fmt::format("Y {:.2f}", zyx[1]), {40, 80}, {0, 0, 255});
//     tools::draw_text(img_with_ypr, fmt::format("X {:.2f}", zyx[2]), {40, 120}, {0, 0, 255});

//     std::vector<cv::Point2f> centers_2d;
//     auto success = cv::findCirclesGrid(img, cv::Size(8, 5), centers_2d);  // 默认是对称圆点图案
//     cv::drawChessboardCorners(img_with_ypr, cv::Size(8, 5), centers_2d, success);  // 显示识别结果
//     cv::resize(img_with_ypr, img_with_ypr, {}, 0.5, 0.5);  // 显示时缩小图片尺寸

//     // 按"s"保存图片和对应四元数，按"q"退出程序
//     cv::imshow("Press s to save, q to quit", img_with_ypr);
//     auto key = cv::waitKey(1);
//     if (key == 'q')
//       break;
//     else if (key != 's')
//       continue;

//     // 保存图片和四元数
//     count++;
//     auto img_path = fmt::format("{}/{}.jpg", output_folder, count);
//     auto q_path = fmt::format("{}/{}.txt", output_folder, count);
//     cv::imwrite(img_path, img);
//     write_q(q_path, q);
//     tools::logger()->info("[{}] Saved in {}", count, output_folder);
//   }

//   // 离开该作用域时，camera和gimbal会自动关闭
// }

// int main(int argc, char * argv[])
// {
//   // 读取命令行参数
//   cv::CommandLineParser cli(argc, argv, keys);
//   if (cli.has("help")) {
//     cli.printMessage();
//     return 0;
//   }
//   auto config_path = cli.get<std::string>(0);
//   auto output_folder = cli.get<std::string>("output-folder");

//   // 新建输出文件夹
//   std::filesystem::create_directory(output_folder);

//   tools::logger()->info("标定板尺寸为8列5行");
//   // 主循环，保存图片和对应四元数
//   capture_loop(config_path, output_folder);  // 移除了can参数

//   tools::logger()->warn("注意四元数输出顺序为wxyz");

//   return 0;
// }
#include <fmt/core.h>
#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "io/camera.hpp"
#include "io/ros2/gimbal_ros.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ?  |                          | 输出命令行参数说明}"
  "{@config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{output-folder o |      assets/img_with_q   | 输出文件夹路径   }";

void write_q(const std::string q_path, const Eigen::Quaterniond & q)
{
  std::ofstream q_file(q_path);
  Eigen::Vector4d xyzw = q.coeffs();
  q_file << fmt::format("{} {} {} {}", xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
  q_file.close();
}

void capture_loop(
  const std::string & config_path, const std::string & output_folder)
{
  auto gimbal = std::make_shared<io::GimbalROS>();
  gimbal->start_spin();
  io::Camera camera(config_path);
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  const auto config = YAML::LoadFile(config_path);
  const cv::Size patternSize(
    config["pattern_cols"].as<int>() - 1, config["pattern_rows"].as<int>() - 1);
  const float squareSize = config["square_size_mm"].as<float>() / 1000.0f;
  const cv::Scalar borderColor(0, 255, 0); // 绿色边框 (BGR格式)
  const int borderThickness = 2;

  int count = 0;
  while (true) {
    camera.read(img, timestamp);
    Eigen::Quaterniond q = gimbal->q(timestamp);

    // 在图像上显示欧拉角
    auto display_img = img.clone();
    Eigen::Vector3d zyx = tools::eulers(q, 2, 1, 0) * 57.3;
    tools::draw_text(display_img, fmt::format("Z {:.2f}", zyx[0]), {40, 40}, {0, 0, 255});
    tools::draw_text(display_img, fmt::format("Y {:.2f}", zyx[1]), {40, 80}, {0, 0, 255});
    tools::draw_text(display_img, fmt::format("X {:.2f}", zyx[2]), {40, 120}, {0, 0, 255});
    
    // 显示四元数信息
    tools::draw_text(display_img, fmt::format("Q: {:.3f}, {:.3f}, {:.3f}, {:.3f}", 
        q.w(), q.x(), q.y(), q.z()), {40, 160}, {0, 0, 255});

    std::vector<cv::Point2f> corners;
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    
    // 检测棋盘格角点
    bool success = cv::findChessboardCorners(
        gray, patternSize, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | 
        cv::CALIB_CB_NORMALIZE_IMAGE |
        cv::CALIB_CB_FAST_CHECK);
    
    if (success) {
        // 亚像素优化
        cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);
        
        // 绘制角点连线
        cv::drawChessboardCorners(display_img, patternSize, corners, success);
        
        // 获取四个角点：左上、右上、右下、左下
        cv::Point2f topLeft = corners[0];
        cv::Point2f topRight = corners[patternSize.width - 1];
        cv::Point2f bottomRight = corners.back();
        cv::Point2f bottomLeft = corners[corners.size() - patternSize.width];
        
        // 绘制绿色边框
        std::vector<cv::Point> borderPoints = {
            topLeft, topRight, bottomRight, bottomLeft
        };
        cv::polylines(display_img, borderPoints, true, borderColor, borderThickness);
        
        // 在边框上显示尺寸信息
        std::string sizeInfo = fmt::format("{}x{} {}mm", 
            patternSize.width + 1, patternSize.height + 1, 
            static_cast<int>(squareSize * 1000));
        cv::putText(display_img, sizeInfo, 
                    topLeft + cv::Point2f(0, -10), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, borderColor, 1);
        
        // 显示检测成功信息
        tools::draw_text(display_img, "Detection: SUCCESS", {40, 200}, {0, 255, 0});
    } else {
        // 显示检测失败信息
        tools::draw_text(display_img, "Detection: FAILED", {40, 200}, {0, 0, 255});
    }

    cv::resize(display_img, display_img, {}, 0.5, 0.5);

    // 按"s"保存图片和对应四元数，按"q"退出程序
    cv::imshow("Press s to save, q to quit", display_img);
    auto key = cv::waitKey(1);
    if (key == 'q')
      break;
    else if (key != 's')
      continue;

    // 保存图片和四元数
    count++;
    auto img_path = fmt::format("{}/{}.jpg", output_folder, count);
    auto q_path = fmt::format("{}/{}.txt", output_folder, count);
    cv::imwrite(img_path, img);
    write_q(q_path, q);
    tools::logger()->info("[{}] Saved in {}", count, output_folder);
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto output_folder = cli.get<std::string>("output-folder");

  std::filesystem::create_directory(output_folder);

  const auto config = YAML::LoadFile(config_path);
  tools::logger()->info(
    "棋盘格内部角点 {}×{}，方格边长 {} mm",
    config["pattern_cols"].as<int>() - 1, config["pattern_rows"].as<int>() - 1,
    config["square_size_mm"].as<float>());
  tools::logger()->info("开始采集标定数据...");
  
  capture_loop(config_path, output_folder);

  tools::logger()->warn("注意四元数输出顺序为wxyz");
  tools::logger()->info("程序结束");

  return 0;
}
