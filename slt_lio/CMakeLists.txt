cmake_minimum_required(VERSION 3.5)
project(slt_lio)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
# dense eigen solves (ikf innovation matrix) need optimization, plain colcon
# build leaves CMAKE_BUILD_TYPE empty (no -O flag)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fpermissive)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(slt_common REQUIRED)
find_package(slt_imu_odometry REQUIRED)
find_package(Sophus REQUIRED)
find_package(ikd_tree REQUIRED)

include_directories(include)

# dependencies
set(dependencies
  rclcpp
  slt_common
  slt_imu_odometry
  ikd_tree
)

# lio_node
add_executable(lio_node
  src/lio_initializer.cpp
  src/kalman_filter/eskf.cpp
  src/fastlio.cpp
  src/fastlio2.cpp
  src/lio_node.cpp
  src/lio_main.cpp)
ament_target_dependencies(lio_node ${dependencies})
target_link_libraries(lio_node
  Sophus::Sophus
)

# install
install(TARGETS
    lio_node
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
