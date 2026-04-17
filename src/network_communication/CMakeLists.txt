cmake_minimum_required(VERSION 3.8)
project(network_communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find_package(Catch2 3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(teleop_msgs REQUIRED)
find_package(serial_msgs REQUIRED)
find_package(autonomy_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include)

add_executable(udp_server
  src/server.cpp
)
add_executable(udp_client
  src/client.cpp
)

ament_target_dependencies(udp_server 
  rclcpp 
  rclcpp_action
  std_msgs 
  teleop_msgs
  autonomy_msgs
)

ament_target_dependencies(udp_client 
  rclcpp 
  rclcpp_action
  std_msgs 
  serial_msgs
)

install(TARGETS
  udp_client
  udp_server
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
