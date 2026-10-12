// #include <fmt/core.h>
// #include <yaml-cpp/yaml.h>

// #include <fstream>
// #include <opencv2/opencv.hpp>

// #include "tools/img_tools.hpp"

// const std::string keys =
//   "{help h usage ? |                          | 输出命令行参数说明}"
//   "{config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
//   "{@input-folder  | assets/img_with_q        | 输入文件夹路径   }";

// std::vector<cv::Point3f> centers_3d(const cv::Size & pattern_size, const float center_distance)
// {
//   std::vector<cv::Point3f> centers_3d;

//   for (int i = 0; i < pattern_size.height; i++)
//     for (int j = 0; j < pattern_size.width; j++)
//       centers_3d.push_back({j * center_distance, i * center_distance, 0});

//   return centers_3d;
// }

// void load(
//   const std::string & input_folder, const std::string & config_path, cv::Size & img_size,
//   std::vector<std::vector<cv::Point3f>> & obj_points,
//   std::vector<std::vector<cv::Point2f>> & img_points)
// {
//   // 读取yaml参数
//   auto yaml = YAML::LoadFile(config_path);
//   auto pattern_cols = yaml["pattern_cols"].as<int>();
//   auto pattern_rows = yaml["pattern_rows"].as<int>();
//   auto center_distance_mm = yaml["center_distance_mm"].as<double>();
//   cv::Size pattern_size(pattern_cols, pattern_rows);

//   for (int i = 1; true; i++) {
//     // 读取图片
//     auto img_path = fmt::format("{}/{}.jpg", input_folder, i);
//     auto img = cv::imread(img_path);
//     if (img.empty()) break;

//     // 设置图片尺寸
//     img_size = img.size();

//     // 识别标定板
//     std::vector<cv::Point2f> centers_2d;
//     auto success = cv::findCirclesGrid(img, pattern_size, centers_2d, cv::CALIB_CB_SYMMETRIC_GRID);

//     // 显示识别结果
//     auto drawing = img.clone();
//     cv::drawChessboardCorners(drawing, pattern_size, centers_2d, success);
//     cv::resize(drawing, drawing, {}, 0.5, 0.5);  // 缩小图片尺寸便于显示完全
//     cv::imshow("Press any to continue", drawing);
//     cv::waitKey(0);

//     // 输出识别结果
//     fmt::print("[{}] {}\n", success ? "success" : "failure", img_path);
//     if (!success) continue;

//     // 记录所需的数据
//     img_points.emplace_back(centers_2d);
//     obj_points.emplace_back(centers_3d(pattern_size, center_distance_mm));
//   }
// }

// void print_yaml(const cv::Mat & camera_matrix, const cv::Mat & distort_coeffs, double error)
// {
//   YAML::Emitter result;
//   std::vector<double> camera_matrix_data(
//     camera_matrix.begin<double>(), camera_matrix.end<double>());
//   std::vector<double> distort_coeffs_data(
//     distort_coeffs.begin<double>(), distort_coeffs.end<double>());

//   result << YAML::BeginMap;
//   result << YAML::Comment(fmt::format("重投影误差: {:.4f}px", error));
//   result << YAML::Key << "camera_matrix";
//   result << YAML::Value << YAML::Flow << camera_matrix_data;
//   result << YAML::Key << "distort_coeffs";
//   result << YAML::Value << YAML::Flow << distort_coeffs_data;
//   result << YAML::Newline;
//   result << YAML::EndMap;

//   fmt::print("\n{}\n", result.c_str());
// }

// int main(int argc, char * argv[])
// {
//   // 读取命令行参数
//   cv::CommandLineParser cli(argc, argv, keys);
//   if (cli.has("help")) {
//     cli.printMessage();
//     return 0;
//   }
//   auto input_folder = cli.get<std::string>(0);
//   auto config_path = cli.get<std::string>("config-path");

//   // 从输入文件夹中加载标定所需的数据
//   cv::Size img_size;
//   std::vector<std::vector<cv::Point3f>> obj_points;
//   std::vector<std::vector<cv::Point2f>> img_points;
//   load(input_folder, config_path, img_size, obj_points, img_points);

//   // 相机标定
//   cv::Mat camera_matrix, distort_coeffs;
//   std::vector<cv::Mat> rvecs, tvecs;
//   auto criteria = cv::TermCriteria(
//     cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100,
//     DBL_EPSILON);  // 默认迭代次数(30)有时会导致结果发散，故设为100
//   cv::calibrateCamera(
//     obj_points, img_points, img_size, camera_matrix, distort_coeffs, rvecs, tvecs, cv::CALIB_FIX_K3,
//     criteria);  // 由于视场角较小，不需要考虑k3

//   // 重投影误差
//   double error_sum = 0;
//   size_t total_points = 0;
//   for (size_t i = 0; i < obj_points.size(); i++) {
//     std::vector<cv::Point2f> reprojected_points;
//     cv::projectPoints(
//       obj_points[i], rvecs[i], tvecs[i], camera_matrix, distort_coeffs, reprojected_points);

//     total_points += reprojected_points.size();
//     for (size_t j = 0; j < reprojected_points.size(); j++)
//       error_sum += cv::norm(img_points[i][j] - reprojected_points[j]);
//   }
//   auto error = error_sum / total_points;

//   // 输出yaml
//   print_yaml(camera_matrix, distort_coeffs, error);
// }


#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <opencv2/opencv.hpp>

#include "tools/img_tools.hpp"

const std::string keys =
  "{help h usage ? |                          | 输出命令行参数说明}"
  "{config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{show-images    | true                     | 是否逐张显示角点 }"
  "{@input-folder  | assets/img_with_q        | 输入文件夹路径   }";

// 生成棋盘格角点的3D坐标
std::vector<cv::Point3f> chessboard_corners_3d(const cv::Size & pattern_size, const float square_size)
{
  std::vector<cv::Point3f> corners_3d;

  for (int i = 0; i < pattern_size.height; i++) {
    for (int j = 0; j < pattern_size.width; j++) {
      // 棋盘格角点在Z=0平面上，X和Y按方格尺寸排列
      corners_3d.push_back({j * square_size, i * square_size, 0});
    }
  }

  return corners_3d;
}

void load(
  const std::string & input_folder, const std::string & config_path, cv::Size & img_size,
  std::vector<std::vector<cv::Point3f>> & obj_points,
  std::vector<std::vector<cv::Point2f>> & img_points, bool show_images)
{
  // 读取yaml参数
  auto yaml = YAML::LoadFile(config_path);
  auto pattern_cols = yaml["pattern_cols"].as<int>();
  auto pattern_rows = yaml["pattern_rows"].as<int>();
  auto square_size_mm = yaml["square_size_mm"].as<double>();
  
  // 棋盘格的内角点数量 = 格子数-1
  cv::Size pattern_size(pattern_cols - 1, pattern_rows - 1);

  fmt::print("棋盘格尺寸: {}x{} (内角点)\n", pattern_size.width, pattern_size.height);
  fmt::print("方格尺寸: {:.2f} mm\n", square_size_mm);

  for (int i = 1; true; i++) {
    // 读取图片
    auto img_path = fmt::format("{}/{}.jpg", input_folder, i);
    auto img = cv::imread(img_path);
    if (img.empty()) {
      // 尝试不同的命名格式
      img_path = fmt::format("{}/{:02d}.jpg", input_folder, i);
      img = cv::imread(img_path);
      if (img.empty()) {
        fmt::print("无法读取图像: {}, 停止加载\n", img_path);
        break;
      }
    }

    // 设置图片尺寸
    if (i == 1) {
      img_size = img.size();
      fmt::print("图像尺寸: {}x{}\n", img_size.width, img_size.height);
    }

    // 转为灰度图
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    // 识别棋盘格角点
    std::vector<cv::Point2f> corners_2d;
    
    // 图像预处理以提高检测率
    cv::Mat processed_gray = gray.clone();
    cv::GaussianBlur(processed_gray, processed_gray, cv::Size(5, 5), 1.5);
    cv::equalizeHist(processed_gray, processed_gray);

    bool success = cv::findChessboardCorners(
      processed_gray, pattern_size, corners_2d,
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

    // 如果预处理后没找到，尝试原图
    if (!success) {
      success = cv::findChessboardCorners(
        gray, pattern_size, corners_2d,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
    }

    if (success) {
      // 角点精细化
      cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);
      cv::cornerSubPix(gray, corners_2d, cv::Size(11, 11), cv::Size(-1, -1), criteria);
    }

    if (show_images) {
      auto drawing = img.clone();
      cv::drawChessboardCorners(drawing, pattern_size, corners_2d, success);
      if (success) {
        tools::draw_text(drawing, fmt::format("成功检测: {}个角点", corners_2d.size()),
                         {10, 30}, {0, 255, 0});
      } else {
        tools::draw_text(drawing, "检测失败", {10, 30}, {0, 0, 255});
      }
      cv::resize(drawing, drawing, {}, 0.5, 0.5);
      cv::imshow("棋盘格检测 - 按任意键继续", drawing);
      cv::waitKey(0);
    }

    // 输出识别结果
    fmt::print("[{}] {} - 角点: {}\n", 
               success ? "success" : "failure", img_path, corners_2d.size());
    
    if (!success) continue;

    // 记录所需的数据
    img_points.emplace_back(corners_2d);
    obj_points.emplace_back(chessboard_corners_3d(pattern_size, square_size_mm));
    
    fmt::print("    已采集 {} 组数据\n", obj_points.size());
  }
  
  if (show_images) cv::destroyAllWindows();
}

void print_yaml(const cv::Mat & camera_matrix, const cv::Mat & distort_coeffs, double error)
{
  YAML::Emitter result;
  std::vector<double> camera_matrix_data(
    camera_matrix.begin<double>(), camera_matrix.end<double>());
  std::vector<double> distort_coeffs_data(
    distort_coeffs.begin<double>(), distort_coeffs.end<double>());

  result << YAML::BeginMap;
  result << YAML::Comment(fmt::format("重投影误差: {:.4f}px", error));
  result << YAML::Key << "camera_matrix";
  result << YAML::Value << YAML::Flow << camera_matrix_data;
  result << YAML::Key << "distort_coeffs";
  result << YAML::Value << YAML::Flow << distort_coeffs_data;
  result << YAML::Newline;
  result << YAML::EndMap;

  fmt::print("\n{}\n", result.c_str());
}

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto input_folder = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto show_images = cli.get<bool>("show-images");

  fmt::print("=== 开始相机内参标定 ===\n");
  fmt::print("输入文件夹: {}\n", input_folder);
  fmt::print("配置文件: {}\n", config_path);

  // 从输入文件夹中加载标定所需的数据
  cv::Size img_size;
  std::vector<std::vector<cv::Point3f>> obj_points;
  std::vector<std::vector<cv::Point2f>> img_points;
  load(input_folder, config_path, img_size, obj_points, img_points, show_images);

  if (obj_points.size() < 8) {
    fmt::print("❌ 有效数据不足 ({}组)，至少需要8组数据进行标定\n", obj_points.size());
    return -1;
  }

  fmt::print("\n开始标定计算...\n");
  fmt::print("有效图像数量: {}\n", obj_points.size());

  // 相机标定
  cv::Mat camera_matrix, distort_coeffs;
  std::vector<cv::Mat> rvecs, tvecs;
  
  // 设置标定参数
  cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON);
  
  // 标定标志 - 根据相机特性调整
  int flags = cv::CALIB_FIX_K3;  // 忽略k3畸变系数，对于大多数相机足够
  
  // 如果需要固定宽高比，可以添加：flags |= cv::CALIB_FIX_ASPECT_RATIO;
  
  double error = cv::calibrateCamera(
    obj_points, img_points, img_size, camera_matrix, distort_coeffs, rvecs, tvecs, flags, criteria);

  // 输出标定结果详情
  fmt::print("\n=== 标定结果详情 ===\n");
  fmt::print("焦距: fx={:.2f}, fy={:.2f}\n", 
             camera_matrix.at<double>(0,0), camera_matrix.at<double>(1,1));
  fmt::print("主点: cx={:.2f}, cy={:.2f}\n",
             camera_matrix.at<double>(0,2), camera_matrix.at<double>(1,2));
  fmt::print("畸变系数: k1={:.6f}, k2={:.6f}, p1={:.6f}, p2={:.6f}, k3={:.6f}\n",
             distort_coeffs.at<double>(0), distort_coeffs.at<double>(1),
             distort_coeffs.at<double>(2), distort_coeffs.at<double>(3),
             distort_coeffs.at<double>(4));
  fmt::print("重投影误差: {:.4f} 像素\n", error);

  // 输出yaml格式结果
  print_yaml(camera_matrix, distort_coeffs, error);
  
  fmt::print("标定完成！\n");


  return 0;
}
