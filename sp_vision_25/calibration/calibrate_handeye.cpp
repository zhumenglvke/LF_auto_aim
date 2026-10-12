

#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <fstream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "tools/img_tools.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                          | 输出命令行参数说明}"
  "{config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{show-images    | true                     | 是否逐张显示角点 }"
  "{@input-folder  | assets/img_with_q        | 输入文件夹路径   }";

// 生成棋盘格角点的3D坐标
std::vector<cv::Point3f> chessboard_corners_3d(const cv::Size & pattern_size, float square_size)
{
  std::vector<cv::Point3f> corners;
  for (int i = 0; i < pattern_size.height; i++)
    for (int j = 0; j < pattern_size.width; j++)
      corners.emplace_back(j * square_size, i * square_size, 0);
  return corners;
}

Eigen::Quaterniond read_q(const std::string & q_path)
{
  std::ifstream q_file(q_path);
  double w, x, y, z;
  q_file >> w >> x >> y >> z;
  return Eigen::Quaterniond(w, x, y, z);
}

void parse_camera_params(const YAML::Node& yaml,
                         cv::Matx33d & camera_matrix, cv::Mat & distort_coeffs)
{
  try {
    // 直接从当前配置文件读取相机参数
    auto cm = yaml["camera_matrix"].as<std::vector<double>>();
    auto dc = yaml["distort_coeffs"].as<std::vector<double>>();

    // 验证数据
    if (cm.size() != 9) {
      throw std::runtime_error("相机内参矩阵必须包含9个元素");
    }
    if (dc.size() != 5) {
      throw std::runtime_error(fmt::format("畸变系数必须包含5个元素，实际有{}个", dc.size()));
    }

    camera_matrix = cv::Matx33d(cm.data());
    distort_coeffs = cv::Mat(dc).reshape(1, 1);
    
    // 详细输出用于调试
    fmt::print("=== 从配置文件加载相机参数 ===\n");
    fmt::print("内参矩阵:\n");
    for (int i = 0; i < 3; i++) {
        fmt::print("[ {:.5f}, {:.5f}, {:.5f} ]\n", 
                   camera_matrix(i, 0), camera_matrix(i, 1), camera_matrix(i, 2));
    }
    fmt::print("畸变系数: [ ");
    for (size_t i = 0; i < dc.size(); i++) {
        fmt::print("{:.5f} ", dc[i]);
    }
    fmt::print("]\n");
    fmt::print("棋盘格尺寸: {}x{}\n", yaml["pattern_cols"].as<int>()-1, yaml["pattern_rows"].as<int>()-1);
    fmt::print("方格尺寸: {:.2f} mm\n", yaml["square_size_mm"].as<double>());
    fmt::print("===================================\n");
  } catch (const YAML::Exception& e) {
    fmt::print("❌ 解析相机参数失败: {}\n", e.what());
    throw;
  } catch (const std::exception& e) {
    fmt::print("❌ 加载相机参数失败: {}\n", e.what());
    throw;
  }
}

// 检查矩阵是否包含NaN或Inf
bool contains_invalid(const cv::Mat& m) {
    for (int i = 0; i < m.rows; i++) {
        for (int j = 0; j < m.cols; j++) {
            double val = m.at<double>(i, j);
            if (std::isnan(val) || std::isinf(val)) {
                return true;
            }
        }
    }
    return false;
}
void verify_coordinate_system(const cv::Mat& tvec, const Eigen::Vector3d& gimbal_ypr, int image_index) {
    fmt::print("\n=== 坐标系验证 (图像 {}) ===\n", image_index);
    
    // 1. 检查OpenCV检测结果
    fmt::print("OpenCV检测结果:\n");
    fmt::print("  tvec: [{:.2f}, {:.2f}, {:.2f}] mm\n", 
               tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
    
    // 2. 检查云台姿态
    fmt::print("云台姿态 (FLU坐标系):\n");
    fmt::print("  yaw={:.2f}°, pitch={:.2f}°, roll={:.2f}°\n", 
               gimbal_ypr[0], gimbal_ypr[1], gimbal_ypr[2]);
    
    // 3. 物理一致性检查
    bool physical_consistency = true;
    std::vector<std::string> inconsistencies;
    
    // 检查标定板位置
    if (tvec.at<double>(2) <= 0) {
        physical_consistency = false;
        inconsistencies.push_back("标定板在相机后方 (tvec.z <= 0)");
    }
    
    // 检查云台姿态与标定板位置的物理关系
    // 如果云台朝前且标定板在相机前方，应该是合理的情况
    if (std::abs(gimbal_ypr[0]) > 90 || std::abs(gimbal_ypr[1]) > 90) {
        physical_consistency = false;
        inconsistencies.push_back("云台姿态异常 (角度过大)");
    }
    
    // 检查重力方向验证
    // 这里可以添加基于四元数的重力方向验证
    
    // 输出验证结果
    if (physical_consistency) {
        fmt::print("✅ 物理姿态一致\n");
    } else {
        fmt::print("❌ 物理姿态不一致！可能存在坐标系混淆\n");
        for (const auto& issue : inconsistencies) {
            fmt::print("   - {}\n", issue);
        }
    }
}
void load(const std::string & input_folder, const std::string & config_path,
          std::vector<double> & R_gimbal2imubody_data,
          std::vector<cv::Mat> & R_gimbal2world_list,
          std::vector<cv::Mat> & t_gimbal2world_list,
          std::vector<cv::Mat> & rvecs, std::vector<cv::Mat> & tvecs, bool show_images)
{
  YAML::Node yaml = YAML::LoadFile(config_path);
  auto pattern_cols = yaml["pattern_cols"].as<int>();
  auto pattern_rows = yaml["pattern_rows"].as<int>();
  auto square_size_mm = yaml["square_size_mm"].as<double>();
  R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();

  cv::Matx33d camera_matrix;
  cv::Mat distort_coeffs;
  
  // 直接从当前配置文件读取相机参数，不再从外部文件读取
  parse_camera_params(yaml, camera_matrix, distort_coeffs);

  const cv::Size pattern_size(pattern_cols - 1, pattern_rows - 1);
  const float square_size = square_size_mm;

  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R_gimbal2imubody(R_gimbal2imubody_data.data());

  int success_count = 0;
  int total_count = 0;
  int invalid_pose = 0;

  fmt::print("开始加载数据...\n");

  for (int i = 1;; i++) {
    auto img_path = fmt::format("{}/{}.jpg", input_folder, i);
    auto q_path = fmt::format("{}/{}.txt", input_folder, i);

    cv::Mat img = cv::imread(img_path);
    if (img.empty()) {
        // 尝试不同命名格式
        img_path = fmt::format("{}/{:02d}.jpg", input_folder, i);
        img = cv::imread(img_path);
        if (img.empty()) {
            fmt::print("无法读取图像: {}, 停止加载\n", img_path);
            break;
        }
    }

    total_count++;
    Eigen::Quaterniond q = read_q(q_path);
    Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
    Eigen::Matrix3d R_gimbal2world =
      R_gimbal2imubody.transpose() * R_imubody2imuabs * R_gimbal2imubody;

    Eigen::Vector3d ypr = tools::eulers(R_gimbal2world, 2, 1, 0) * 57.3;
    if (std::isnan(ypr[0]) || std::isnan(ypr[1])) {
      fmt::print("⚠️ 第 {} 张：IMU 姿态异常（含 NaN）\n", i);
      invalid_pose++;
      continue;
    }

    fmt::print("第 {:02d} 张 -> yaw:{:6.2f}°, pitch:{:6.2f}°, roll:{:6.2f}°\n",
               i, ypr[0], ypr[1], ypr[2]);

    std::vector<cv::Point2f> corners_2d;
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    
    // 图像预处理
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    bool found = cv::findChessboardCorners(
      gray, pattern_size, corners_2d,
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

    fmt::print("    found = {} | corners = {}\n", found, corners_2d.size());

    if (!found) {
      fmt::print("    ❌ 棋盘格未检测到\n");
      continue;
    }

    cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);
    cv::cornerSubPix(gray, corners_2d, cv::Size(11, 11), cv::Size(-1, -1), criteria);

    if (show_images) {
      cv::Mat img_display = img.clone();
      cv::drawChessboardCorners(img_display, pattern_size, corners_2d, found);
      cv::imshow("Chessboard Corners", img_display);
      cv::waitKey(100);
    }

    std::vector<cv::Point3f> corners_3d = chessboard_corners_3d(pattern_size, square_size);

    cv::Mat rvec, tvec;
    bool pnp_ok = cv::solvePnP(
      corners_3d, corners_2d, camera_matrix, distort_coeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);

    // 检查结果是否有效
    if (!pnp_ok || contains_invalid(rvec) || contains_invalid(tvec)) {
      fmt::print("    ⚠️ solvePnP 结果异常\n");
      if (pnp_ok) {
        fmt::print("        rvec: [{:.2f}, {:.2f}, {:.2f}]\n", 
                   rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));
        fmt::print("        tvec: [{:.2f}, {:.2f}, {:.2f}]\n", 
                   tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
      }
      continue;
    }

    fmt::print("    ✅ 棋盘格检测成功 | tvec = [{:.2f}, {:.2f}, {:.2f}] mm\n",
               tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
    verify_coordinate_system(tvec, ypr, i);
    cv::Mat R_cv;
    cv::eigen2cv(R_gimbal2world, R_cv);
    cv::Mat t_gimbal2world = (cv::Mat_<double>(3, 1) << 0, 0, 0);

    R_gimbal2world_list.push_back(R_cv);
    t_gimbal2world_list.push_back(t_gimbal2world);
    rvecs.push_back(rvec);
    tvecs.push_back(tvec);
    success_count++;
  }

  if (show_images) cv::destroyAllWindows();

  fmt::print("\n========== 数据加载完成 ==========\n");
  fmt::print("总图像数：{}\n", total_count);
  fmt::print("棋盘格识别成功：{}\n", success_count);
  fmt::print("IMU 姿态异常：{}\n", invalid_pose);
  fmt::print("有效样本（用于标定）：{}\n\n", success_count);

  if (success_count < 5)
    fmt::print("⚠️ 样本太少，建议至少拍摄 8-10 张不同角度图像\n");
}

void print_yaml(const std::vector<double> & R_gimbal2imubody_data,
                const cv::Mat & R_camera2gimbal, const cv::Mat & t_camera2gimbal,
                const Eigen::Vector3d & ypr)
{
  YAML::Emitter out;
  std::vector<double> R_data(R_camera2gimbal.begin<double>(), R_camera2gimbal.end<double>());
  std::vector<double> t_data(t_camera2gimbal.begin<double>(), t_camera2gimbal.end<double>());

  out << YAML::BeginMap;
  out << YAML::Key << "R_gimbal2imubody" << YAML::Value << YAML::Flow << R_gimbal2imubody_data;
  out << YAML::Comment(fmt::format(" 相机偏角: yaw {:.2f}°, pitch {:.2f}°, roll {:.2f}° ", ypr[0], ypr[1], ypr[2]));
  out << YAML::Key << "R_camera2gimbal" << YAML::Value << YAML::Flow << R_data;
  out << YAML::Key << "t_camera2gimbal" << YAML::Value << YAML::Flow << t_data;
  out << YAML::EndMap;

  fmt::print("\n============== 标定结果 ==============\n{}\n", out.c_str());
}

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto input_folder = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto show_images = cli.get<bool>("show-images");

  std::vector<double> R_gimbal2imubody_data;
  std::vector<cv::Mat> R_gimbal2world_list, t_gimbal2world_list;
  std::vector<cv::Mat> rvecs, tvecs;

  load(input_folder, config_path, R_gimbal2imubody_data,
       R_gimbal2world_list, t_gimbal2world_list, rvecs, tvecs, show_images);

  if (R_gimbal2world_list.size() < 5) {
    fmt::print("❌ 有效数据不足，无法进行手眼标定。\n");
    return 0;
  }

  // 检查tvec变化情况
  double tvec_change = 0;
  for (size_t i = 1; i < tvecs.size(); i++) {
    double diff = cv::norm(tvecs[i] - tvecs[i-1]);
    tvec_change += diff;
  }
  double avg_tvec_change = tvec_change / (tvecs.size() - 1);
  fmt::print("平均tvec变化: {:.2f} mm\n", avg_tvec_change);
  
  if (avg_tvec_change < 10) {
    fmt::print("⚠️ 平移变化过小，使用纯旋转标定方法\n");
  }

  cv::Mat R_camera2gimbal, t_camera2gimbal;
  
  // 使用CALIB_HAND_EYE_PARK方法 - 针对纯旋转场景优化
  cv::calibrateHandEye(R_gimbal2world_list, t_gimbal2world_list,
                       rvecs, tvecs, R_camera2gimbal, t_camera2gimbal,
                       cv::CALIB_HAND_EYE_HORAUD);
  
  t_camera2gimbal /= 1e3;  // 毫米转米

  // 计算标定结果的欧拉角
  Eigen::Matrix3d R_cam2gimbal_eigen;
  cv::cv2eigen(R_camera2gimbal, R_cam2gimbal_eigen);
  
  // 理想坐标系定义：X轴向前，Y轴向左，Z轴向上
  Eigen::Matrix3d R_gimbal2ideal{{0, -1, 0}, {0, 0, -1}, {1, 0, 0}};
  Eigen::Matrix3d R_camera2ideal = R_gimbal2ideal * R_cam2gimbal_eigen;
  
  // 计算欧拉角（YPR顺序）
  Eigen::Vector3d ypr = tools::eulers(R_camera2ideal, 1, 0, 2) * 57.3;

  print_yaml(R_gimbal2imubody_data, R_camera2gimbal, t_camera2gimbal, ypr);
  
  // 添加标定质量评估
  fmt::print("\n========== 标定质量评估 ==========\n");
  fmt::print("旋转矩阵行列式: {:.6f} (应接近1)\n", cv::determinant(R_camera2gimbal));
  return 0;
}
