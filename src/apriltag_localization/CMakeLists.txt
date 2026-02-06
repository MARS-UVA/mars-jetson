cmake_minimum_required(VERSION 3.8)
project(apriltag_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(GTSAM REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(BUILD_EXAMPLES OFF)
set(INSTALL_PKG_CONFIG OFF)

add_subdirectory(vendor/mars-apriltag-localizer)

include_directories(
  include/${PROJECT_NAME}
  vendor/mars-apriltag-localizer/include/marslocalizer
)

add_executable(${PROJECT_NAME}
  src/node.cpp
)
add_dependencies(${PROJECT_NAME}
  marslocalizer
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

target_link_libraries(${PROJECT_NAME}
  marslocalizer
  rclcpp::rclcpp
  image_transport::image_transport
  cv_bridge::cv_bridge
  tf2_eigen::tf2_eigen
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  gtsam
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
