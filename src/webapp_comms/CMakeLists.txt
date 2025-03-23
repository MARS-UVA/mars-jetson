cmake_minimum_required(VERSION 3.8)
project(webapp_comms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(Catch2 3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(teleop_msgs REQUIRED)

find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include)

add_executable(net_node
  src/net_node.cpp
  src/server.cpp
  src/client.cpp
  src/frame_sender.cpp
)

target_link_libraries(net_node ${OpenCV_LIBS})
target_include_directories(net_node PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(net_node 
  rclcpp 
  std_msgs 
  sensor_msgs 
  cv_bridge 
  image_transport 
  OpenCV 
  teleop_msgs
  rosidl_runtime_cpp
  rosidl_typesupport_introspection_cpp
  rosidl_runtime_cpp
)

install(TARGETS
  net_node
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