cmake_minimum_required(VERSION 3.5)
project(slt_lio_mapping)

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
find_package(g2o REQUIRED
  COMPONENTS
  g2o::core
  g2o::types_slam3d
  g2o::solver_cholmod
  g2o::solver_csparse
  g2o::solver_pcg
)
find_package(Sophus REQUIRED)

include_directories(include)

# libaray
set(dependencies
  rclcpp
  slt_interface
  slt_common
  slt_imu_odometry
)

add_library(${PROJECT_NAME} SHARED
  # graph_optimizer
  src/graph_optimizer/g2o_graph_optimizer.cpp
  # lio_backend
  src/lio_back_end.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME}
  g2o::core
  g2o::types_slam3d
  g2o::solver_cholmod
  g2o::solver_csparse
  g2o::solver_pcg
  Sophus::Sophus
)

# lio_back_end_node
add_executable(lio_back_end_node
  src/lio_back_end_node.cpp
  src/lio_back_end_main.cpp)
target_link_libraries(lio_back_end_node ${PROJECT_NAME})

# install include directories
install(DIRECTORY include/
  DESTINATION include
)

# install libraries
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# install
install(TARGETS
    lio_back_end_node
  DESTINATION lib/${PROJECT_NAME}
)

# install directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_export_dependencies(g2o)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
