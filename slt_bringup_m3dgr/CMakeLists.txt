cmake_minimum_required(VERSION 3.5)
project(slt_bringup_m3dgr)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic -fpermissive)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(slt_common REQUIRED)
find_package(yaml-cpp REQUIRED)

set(dependencies
  rclcpp
  nav_msgs
  tf2_ros
  slt_common
)

include_directories(include)

# replay_node
add_executable(replay_node
  src/replay_node.cpp
  src/replay_main.cpp)
ament_target_dependencies(replay_node ${dependencies})
target_link_libraries(replay_node ${YAML_CPP_LIBRARIES})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS replay_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/rosbag1_to_rosbag2.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
