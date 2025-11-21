cmake_minimum_required(VERSION 3.8)
project(network-comms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package (rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


add_executable(net_node
  src/net-node.cpp
  src/server.cpp
  src/client.cpp
)
add_executable(string_publisher
  src/string-publisher.cpp
)

target_link_libraries(net_node PUBLIC
      ${std_msgs_TARGETS}
      rclcpp::rclcpp
)
target_link_libraries(string_publisher PUBLIC
      ${std_msgs_TARGETS}
      rclcpp::rclcpp
)

install(TARGETS 
  net_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS 
  string_publisher
  DESTINATION lib/${PROJECT_NAME}
)


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
