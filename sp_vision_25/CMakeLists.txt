cmake_minimum_required(VERSION 3.16.3)

project(sp_vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/")
find_package(OpenVINO REQUIRED)

option(SP_VISION_AUTO_AIM_ONLY "Build standalone auto-aim tests without ROS targets" OFF)
if(SP_VISION_AUTO_AIM_ONLY)
    include_directories(${EIGEN3_INCLUDE_DIR})
    include_directories(${OpenCV_INCLUDE_DIRS})
    include_directories(${PROJECT_SOURCE_DIR})

    add_subdirectory(tools)
    add_subdirectory(io)
    add_subdirectory(tasks/auto_aim)

    add_executable(auto_aim_test tests/auto_aim_test.cpp)
    target_link_libraries(auto_aim_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
    add_executable(auto_aim_camera_test tests/auto_aim_camera_test.cpp)
    target_link_libraries(auto_aim_camera_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
    add_executable(auto_aim_test_all tests/auto_aim_test_all.cpp)
    target_link_libraries(auto_aim_test_all ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)

    option(SP_VISION_BUILD_ROS_AUTO_AIM "Build the single-camera ROS auto-aim executable" OFF)
    if(SP_VISION_BUILD_ROS_AUTO_AIM)
        find_package(ament_cmake REQUIRED)
        find_package(rclcpp REQUIRED)
        find_package(sensor_msgs REQUIRED)
        find_package(std_msgs REQUIRED)
        find_package(geometry_msgs REQUIRED)
        find_package(example_interfaces REQUIRED)
        find_package(tf2 REQUIRED)
        find_package(tf2_geometry_msgs REQUIRED)
        find_package(tf2_ros REQUIRED)
        find_package(pb_rm_interfaces REQUIRED)

        add_executable(auto_aim_test_all_ros
            tests/auto_aim_test_all_ros.cpp
            io/ros2/gimbal_ros.cpp
        )
        target_link_libraries(auto_aim_test_all_ros
            ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim
            rclcpp::rclcpp tf2::tf2 tf2_geometry_msgs::tf2_geometry_msgs
            tf2_ros::tf2_ros pb_rm_interfaces::pb_rm_interfaces__rosidl_typesupport_cpp
        )
        ament_target_dependencies(auto_aim_test_all_ros
            rclcpp sensor_msgs std_msgs geometry_msgs example_interfaces
            tf2 tf2_geometry_msgs tf2_ros pb_rm_interfaces
        )

        add_executable(capture_gimbal
            calibration/capture_gimbal.cpp
            io/ros2/gimbal_ros.cpp
        )
        target_link_libraries(capture_gimbal
            ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io
            rclcpp::rclcpp tf2::tf2 tf2_geometry_msgs::tf2_geometry_msgs
            tf2_ros::tf2_ros pb_rm_interfaces::pb_rm_interfaces__rosidl_typesupport_cpp
        )
        ament_target_dependencies(capture_gimbal
            rclcpp sensor_msgs std_msgs geometry_msgs example_interfaces
            tf2 tf2_geometry_msgs tf2_ros pb_rm_interfaces
        )

        add_executable(capture calibration/capture.cpp)
        target_link_libraries(capture ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)

        add_executable(calibrate_camera calibration/calibrate_camera.cpp)
        target_link_libraries(calibrate_camera ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)

        add_executable(calibrate_handeye calibration/calibrate_handeye.cpp)
        target_link_libraries(calibrate_handeye
            ${OpenCV_LIBS} fmt::fmt yaml-cpp tools
        )
    endif()
    return()
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)      # 处理 /serial/imu
set(pb_rm_interfaces_DIR
  /home/luojiafox/Desktop/pb2025_sentry_ws/install/pb_rm_interfaces/share/pb_rm_interfaces/cmake
)
find_package(pb_rm_interfaces REQUIRED)          # 自定义消息
find_package(tf2 REQUIRED)              # 用于四元数与欧拉角转换
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_cmake REQUIRED)
find_package(example_interfaces REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})


add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_aim)
add_subdirectory(tasks/auto_buff)
add_subdirectory(tasks/omniperception)

####################src###################
add_executable(standard src/standard.cpp)
add_executable(mt_standard src/mt_standard.cpp)
add_executable(standard_mpc src/standard_mpc.cpp)
add_executable(auto_aim_debug_mpc src/auto_aim_debug_mpc.cpp)
add_executable(mt_auto_aim_debug src/mt_auto_aim_debug.cpp)
add_executable(auto_buff_debug 
    src/auto_buff_debug.cpp
    io/ros2/gimbal_ros.cpp                  # 添加新的 ROS 包装类源文件
)
target_link_libraries(auto_buff_debug
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES}
    glog
    gflags
    fmt::fmt
    yaml-cpp
    tools
    io
    auto_buff
    # --- 新增 ROS 2 相关库链接 ---
    rclcpp::rclcpp
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
    pb_rm_interfaces::pb_rm_interfaces__rosidl_typesupport_cpp
)
ament_target_dependencies(auto_buff_debug
    rclcpp
    sensor_msgs
    std_msgs
    tf2
    tf2_geometry_msgs
    example_interfaces
    pb_rm_interfaces
)
add_executable(auto_buff_debug_mpc src/auto_buff_debug_mpc.cpp)
add_executable(uav src/uav.cpp)
add_executable(uav_debug src/uav_debug.cpp)
add_executable(sentry_dual_cameras
  src/sentry_dual_cameras.cpp
  tasks/omniperception/decider.cpp   
)
add_executable(sentry_dual_sn
  src/sentry_dual_sn.cpp
  tasks/omniperception/decider.cpp   
)
add_executable(sentry_bp_test
  src/sentry_bp_test.cpp
  tasks/omniperception/decider.cpp   
)
add_executable(sentry_dual_cameras_ros
  src/sentry_dual_cameras_ros.cpp
  io/ros2/gimbal_ros.cpp                  # 添加新的 ROS 包装类源文件
  tasks/omniperception/decider.cpp   
)

add_executable(dual_camera_test tests/dual_camera_test.cpp)

target_link_libraries(dual_camera_test PRIVATE 
    tools 
    fmt::fmt 
    ${OpenCV_LIBS}
)
# 链接必要的模块
target_link_libraries(sentry_dual_cameras 
    ${OpenCV_LIBS} 
    fmt::fmt 
    yaml-cpp 
    auto_aim    # 包含 YOLO, Tracker, Solver [cite: 1]
    tools       # 包含 Logger, MathTools [cite: 1]
    io          # 包含 Camera, Gimbal [cite: 1]
)
target_link_libraries(sentry_dual_sn
    ${OpenCV_LIBS} 
    fmt::fmt 
    yaml-cpp 
    auto_aim    # 包含 YOLO, Tracker, Solver [cite: 1]
    tools       # 包含 Logger, MathTools [cite: 1]
    io          # 包含 Camera, Gimbal [cite: 1]
)
target_link_libraries(sentry_bp_test
    ${OpenCV_LIBS} 
    fmt::fmt 
    yaml-cpp 
    auto_aim    # 包含 YOLO, Tracker, Solver [cite: 1]
    tools       # 包含 Logger, MathTools [cite: 1]
    io          # 包含 Camera, Gimbal [cite: 1]
)
target_link_libraries(sentry_dual_cameras_ros
    ${OpenCV_LIBS} 
    fmt::fmt 
    yaml-cpp 
    auto_aim
    tools
    io
    rclcpp::rclcpp
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
    tf2_ros::tf2_ros
)
ament_target_dependencies(sentry_dual_cameras_ros
  rclcpp
  sensor_msgs
  pb_rm_interfaces
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  example_interfaces
)

target_link_libraries(standard ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(mt_standard ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(standard_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(auto_aim_debug_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(mt_auto_aim_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(auto_buff_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io ${CERES_LIBRARIES})
target_link_libraries(auto_buff_debug_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io ${CERES_LIBRARIES})
target_link_libraries(uav ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(uav_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

##################calibration################
add_executable(capture_gimbal 
    calibration/capture_gimbal.cpp
    io/ros2/gimbal_ros.cpp                  # 确保这里包含了 ROS 包装类的实现
    )
add_executable(capture calibration/capture.cpp)
add_executable(calibrate_camera calibration/calibrate_camera.cpp)
add_executable(calibrate_handeye calibration/calibrate_handeye.cpp)
add_executable(calibrate_robotworld_handeye calibration/calibrate_robotworld_handeye.cpp)
add_executable(split_video calibration/split_video.cpp)

target_link_libraries(capture_gimbal ${OpenCV_LIBS} fmt::fmt tools io
    # --- 新增 ROS 2 相关库链接 ---
    rclcpp::rclcpp
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
    pb_rm_interfaces::pb_rm_interfaces__rosidl_typesupport_cpp
)
ament_target_dependencies(capture_gimbal
    rclcpp
    sensor_msgs  # 这是关键，添加 sensor_msgs 依赖
    pb_rm_interfaces
    std_msgs
    tf2
    tf2_geometry_msgs
    example_interfaces
)
target_link_libraries(capture ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(calibrate_camera ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(calibrate_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(calibrate_robotworld_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(split_video ${OpenCV_LIBS} fmt::fmt tools)

##################tests##################
add_executable(auto_aim_test tests/auto_aim_test.cpp)
add_executable(auto_aim_multiCamera_test tests/auto_aim_multiCamera_test.cpp)
add_executable(auto_buff_test tests/auto_buff_test.cpp)
add_executable(camera_detect_test tests/camera_detect_test.cpp)
add_executable(camera_test tests/camera_test.cpp)
add_executable(camera_thread_test tests/camera_thread_test.cpp)
add_executable(cboard_test tests/cboard_test.cpp)
add_executable(fire_test tests/fire_test.cpp)
add_executable(detector_video_test tests/detector_video_test.cpp)
add_executable(gimbal_response_test tests/gimbal_response_test.cpp)
add_executable(multi_usbcamera_test tests/multi_usbcamera_test.cpp)
add_executable(usbcamera_detect_test tests/usbcamera_detect_test.cpp)
add_executable(usbcamera_test tests/usbcamera_test.cpp)
add_executable(handeye_test tests/handeye_test.cpp)
add_executable(dm_test tests/dm_test.cpp)
add_executable(minimum_vision_system tests/minimum_vision_system.cpp)
add_executable(vision_system tests/vision_system.cpp)
add_executable(auto_aim_mutithread tests/auto_aim_mutithread.cpp)

add_executable(auto_aim_test_all tests/auto_aim_test_all.cpp)
add_executable(auto_aim_test_all_ros 
    tests/auto_aim_test_all_ros.cpp
    io/ros2/gimbal_ros.cpp                  # 确保这里包含了 ROS 包装类的实现
)
add_executable(auto_aim_mycamera_test
    tests/auto_aim_mycamera_test.cpp
)
add_executable(My_gimbal_test
    tests/My_gimbal_test.cpp
)
add_executable(auto_aim_camera_test
    tests/auto_aim_camera_test.cpp
)
target_link_libraries(My_gimbal_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)

target_link_libraries(auto_aim_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(auto_buff_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io)
target_link_libraries(camera_detect_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim io)
target_link_libraries(camera_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(camera_thread_test ${OpenCV_LIBS} fmt::fmt auto_aim tools io)
target_link_libraries(cboard_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(fire_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(detector_video_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim)
target_link_libraries(gimbal_response_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
target_link_libraries(multi_usbcamera_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(usbcamera_detect_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(usbcamera_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
target_link_libraries(handeye_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(dm_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
target_link_libraries(minimum_vision_system ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(vision_system ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(auto_aim_test_all ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim auto_buff io)
target_link_libraries(auto_aim_test_all_ros
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES}
    glog
    gflags
    fmt::fmt
    yaml-cpp
    tools
    io
    auto_aim
    # --- 新增 ROS 2 相关库链接 ---
    rclcpp::rclcpp
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs    
    tf2_ros::tf2_ros
    pb_rm_interfaces::pb_rm_interfaces__rosidl_typesupport_cpp
)
ament_target_dependencies(auto_aim_test_all_ros
    rclcpp
    sensor_msgs
    std_msgs
    tf2
    tf2_geometry_msgs    
    tf2_ros
    example_interfaces
    pb_rm_interfaces
)
target_link_libraries(auto_aim_mutithread ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(auto_aim_mycamera_test
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES}
    glog
    gflags fmt::fmt yaml-cpp tools io auto_aim
)

target_link_libraries(auto_aim_camera_test
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES}
    glog
    gflags fmt::fmt yaml-cpp tools io auto_aim
)
# 为你的双相机测试程序添加链接库
target_link_libraries(auto_aim_multiCamera_test
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES}
    glog
    gflags
    fmt::fmt
    yaml-cpp
    tools
    io
    auto_aim
)

add_executable(gimbal_test tests/gimbal_test.cpp)
target_link_libraries(gimbal_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

add_executable(planner_test tests/planner_test.cpp)
target_link_libraries(planner_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

add_executable(planner_test_offline tests/planner_test_offline.cpp)
target_link_libraries(planner_test_offline ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

add_executable(sentry_dual_sn_ros 
    src/sentry_dual_sn_ros.cpp
    io/ros2/gimbal_ros.cpp
    tasks/omniperception/decider.cpp
)

target_link_libraries(sentry_dual_sn_ros
    ${OpenCV_LIBS}
    fmt::fmt
    yaml-cpp
    auto_aim
    tools
    io
    rclcpp::rclcpp
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs    
    tf2_ros::tf2_ros
    pb_rm_interfaces::pb_rm_interfaces__rosidl_typesupport_cpp  # 处理之前报错的自定义消息
)

ament_target_dependencies(sentry_dual_sn_ros
    rclcpp
    sensor_msgs
    std_msgs
    tf2
    tf2_geometry_msgs    
    tf2_ros
    pb_rm_interfaces
    example_interfaces
)

# 检查 ROS 环境
# find_package(ament_cmake QUIET)
# find_package(rclcpp QUIET)
# find_package(std_msgs QUIET)
# find_package(rosidl_typesupport_cpp QUIET)
find_package(sp_msgs QUIET)
# find_package(rclcpp REQUIRED)

# 只有在找到 ROS 环境时才会添加 sentry 和 publish_test
if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND AND rosidl_typesupport_cpp_FOUND AND sp_msgs_FOUND)
    # 编译 sentry 和 publish_test
    add_executable(sentry src/sentry.cpp)
    add_executable(sentry_bp src/sentry_bp.cpp)
    add_executable(sentry_debug src/sentry_debug.cpp)
    add_executable(sentry_multithread src/sentry_multithread.cpp)
    add_executable(publish_test tests/publish_test.cpp)
    add_executable(subscribe_test tests/subscribe_test.cpp)
    add_executable(topic_loop_test tests/topic_loop_test.cpp)

    target_link_libraries(sentry ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(sentry_bp ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(sentry_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(sentry_multithread ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(publish_test ${OpenCV_LIBS} fmt::fmt tools io auto_aim)
    target_link_libraries(subscribe_test ${OpenCV_LIBS} fmt::fmt tools io)
    target_link_libraries(topic_loop_test ${OpenCV_LIBS} fmt::fmt tools io)
else()
    message(STATUS "ROS2 environment not found, skipping ROS2-related code.")
endif()
