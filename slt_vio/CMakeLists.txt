cmake_minimum_required(VERSION 3.5)
project(slt_vio)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
# optical flow and feature detection need optimization, plain colcon build leaves
# CMAKE_BUILD_TYPE empty (no -O flag)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fpermissive)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(slt_common REQUIRED)
find_package(slt_imu_odometry REQUIRED)
find_package(Sophus REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)

include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})

# dependencies
set(dependencies
  rclcpp
  slt_common
  slt_imu_odometry
  cv_bridge
)

# vio_node
add_executable(vio_node
  src/feature_tracker.cpp
  src/feature_utils.cpp
  src/initializer/global_sfm.cpp
  src/initializer/visual_inertial_initializer.cpp
  src/optimizer/optimizer.cpp
  src/vins_mono.cpp
  src/vio_node.cpp
  src/vio_main.cpp)
ament_target_dependencies(vio_node ${dependencies})
target_link_libraries(vio_node
  Sophus::Sophus
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
)

# install
install(TARGETS
    vio_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
