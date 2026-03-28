cmake_minimum_required(VERSION 3.5)
project(slt_lidar_locator)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fpermissive)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(slt_interface REQUIRED)
find_package(slt_common REQUIRED)
find_package(scan_context REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  slt_interface
  slt_common
  scan_context
)

# slt_lidar_locator_node
add_executable(slt_lidar_locator_node
  src/lidar_locator.cpp
  src/lidar_locator_node.cpp
  src/lidar_locator_main.cpp)

ament_target_dependencies(slt_lidar_locator_node ${dependencies})

# install executable
install(TARGETS
    slt_lidar_locator_node
  DESTINATION lib/${PROJECT_NAME}
)

# install directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
