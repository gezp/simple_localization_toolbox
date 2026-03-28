cmake_minimum_required(VERSION 3.5)
project(slt_eskf_locator)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fpermissive)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Sophus REQUIRED)
find_package(slt_interface REQUIRED)
find_package(slt_common REQUIRED)
# find_package(backward_ros REQUIRED)

include_directories(include)

# libaray
set(dependencies
  ament_index_cpp
  slt_interface
  slt_common
)

add_library(${PROJECT_NAME} SHARED
  src/kalman_filter/eskf.cpp
  src/kalman_filter/imu_integration.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# lidar_imu_fusion_node
add_executable(lidar_imu_fusion_node
  src/lidar_imu_fusion.cpp
  src/lidar_imu_fusion_node.cpp
  src/lidar_imu_fusion_main.cpp)
target_link_libraries(lidar_imu_fusion_node ${PROJECT_NAME})
ament_target_dependencies(lidar_imu_fusion_node)

# install
install(TARGETS
    lidar_imu_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install  directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
