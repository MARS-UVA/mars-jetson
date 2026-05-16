cmake_minimum_required(VERSION 3.8)
project(robot_state_machine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(teleop_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)

add_executable(state_machine
  src/state_machine.cpp
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ResetBreadcrumbing.srv"
)

target_link_libraries(state_machine PUBLIC
  ${PROJECT_NAME}__rosidl_typesupport_cpp
  ${teleop_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
)

target_include_directories(state_machine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS 
  state_machine
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()