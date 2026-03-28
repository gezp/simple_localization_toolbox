cmake_minimum_required(VERSION 3.5)
project(slt_graph_locator)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic -fpermissive)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(slt_interface REQUIRED)
find_package(slt_common REQUIRED)
find_package(slt_imu_odometry REQUIRED)
find_package(Sophus REQUIRED)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
# find_package(backward_ros REQUIRED)

include_directories(include)

# libaray
set(dependencies
  rclcpp
  slt_interface
  slt_common
  slt_imu_odometry
)

add_library(${PROJECT_NAME} SHARED
  src/graph_optimizer/ceres_graph_optimizer.cpp
  src/sliding_window.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES})

# sliding_window_node
add_executable(sliding_window_node
  src/sliding_window_node.cpp
  src/sliding_window_main.cpp)
target_link_libraries(sliding_window_node ${PROJECT_NAME})

# Install libraries
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# install
install(TARGETS
    sliding_window_node
  DESTINATION lib/${PROJECT_NAME}
)

# install directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_export_dependencies(Ceres)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
