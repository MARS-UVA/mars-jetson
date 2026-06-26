cmake_minimum_required(VERSION 3.8)
project(digdump)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(autonomy_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(action_server src/action_server.cpp)
ament_target_dependencies(action_server
  rclcpp
  std_msgs
  sensor_msgs
  robot_control_msgs
  rclcpp_action
  geometry_msgs
  autonomy_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  action_server
  DESTINATION lib/${PROJECT_NAME})


ament_package()
