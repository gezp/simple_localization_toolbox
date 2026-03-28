cmake_minimum_required(VERSION 3.5)
project(slt_lidar_odometry)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fpermissive)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(slt_interface REQUIRED)
find_package(slt_common REQUIRED)
# find_package(backward_ros REQUIRED)

include_directories(include)

# dependencies
set(dependencies
  rclcpp
  slt_interface
  slt_common
)

# lidar_odometry_node
add_executable(lidar_odometry_node
  src/simple_odometry.cpp
  src/loam_odometry.cpp
  src/lidar_odometry_node.cpp
  src/lidar_odometry_main.cpp)
ament_target_dependencies(lidar_odometry_node ${dependencies})

# install
install(TARGETS
    lidar_odometry_node
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
