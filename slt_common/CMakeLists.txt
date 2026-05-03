cmake_minimum_required(VERSION 3.5)
project(slt_common)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic -fpermissive)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(slt_interface REQUIRED)
find_package(yaml-cpp REQUIRED)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)
find_package(Ceres REQUIRED)
find_package(Sophus REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)
# find_package(backward_ros REQUIRED)

include_directories(include)

# libaray
set(dependencies
  rclcpp
  pcl_ros
  tf2_ros
  tf2_eigen
  geometry_msgs
  sensor_msgs
  nav_msgs
  slt_interface
)
add_library(${PROJECT_NAME} SHARED
  # publisher
  src/publisher/cloud_publisher.cpp
  src/publisher/gnss_publisher.cpp
  src/publisher/imu_publisher.cpp
  src/publisher/lidar_frames_publisher.cpp
  src/publisher/path_publisher.cpp
  src/publisher/loop_candidate_publisher.cpp
  src/publisher/odometry_publisher.cpp
  # subscriber
  src/subscriber/cloud_subscriber.cpp
  src/subscriber/gnss_subscriber.cpp
  src/subscriber/imu_subscriber.cpp
  src/subscriber/lidar_frames_subscriber.cpp
  src/subscriber/loop_candidate_subscriber.cpp
  src/subscriber/nav_sat_fix_subscriber.cpp
  src/subscriber/odometry_subscriber.cpp
  src/subscriber/twist_subscriber.cpp
  # point_cloud_filter
  src/point_cloud_filter/box_filter.cpp
  src/point_cloud_filter/voxel_filter.cpp
  # point_cloud_registration
  src/point_cloud_registration/pcl_icp.cpp
  src/point_cloud_registration/icp_svd.cpp
  src/point_cloud_registration/pcl_ndt.cpp
  src/point_cloud_registration/ndt.cpp
  src/point_cloud_registration/ndt/voxel_grid_covariance.cpp
  src/point_cloud_registration/ndt/normal_distributions_transform.cpp
  src/point_cloud_registration/point_cloud_registration_factory.cpp
  # loam
  src/loam/loam_feature_extraction.cpp
  src/loam/loam_registration.cpp
  # utils
  src/msg_utils.cpp
  src/sensor_data_utils.cpp
  src/lidar_key_frame_manager.cpp
  src/lidar_utils.cpp
  src/extrinsics_manager.cpp
  src/odom_data_buffer.cpp
  src/tic_toc.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME}
  ${GeographicLib_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${CERES_LIBRARIES}
  Sophus::Sophus
  ${PCL_LIBRARIES}
)

# kitti_preprocess_node
add_executable(kitti_preprocess_node
  src/kitti_preprocess_node.cpp
  src/kitti_preprocess_main.cpp)

target_link_libraries(kitti_preprocess_node ${PROJECT_NAME})

# simple_evaluator_node
add_executable(simple_evaluator_node
  src/simple_evaluator_node.cpp
  src/simple_evaluator_main.cpp)

target_link_libraries(simple_evaluator_node ${PROJECT_NAME})

# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install libraries
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install executables
install(TARGETS
    kitti_preprocess_node
    simple_evaluator_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_export_dependencies(Ceres)
ament_export_dependencies(Sophus)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
