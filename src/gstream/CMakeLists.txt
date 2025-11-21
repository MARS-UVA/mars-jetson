cmake_minimum_required(VERSION 3.8)
project(gstream)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED IMPORTED_TARGET gstreamer-1.0>=1.14)
pkg_check_modules(GSTREAMER_APP REQUIRED IMPORTED_TARGET gstreamer-app-1.0)

add_executable(gstreamer_node
  src/streamer.cpp
)

target_compile_features(gstreamer_node PUBLIC c_std_99 cxx_std_17)

target_include_directories(gstreamer_node PUBLIC
  ${sensor_msgs_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
  ${GSTREAMER_APP_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
)

target_link_libraries(gstreamer_node
  rclcpp::rclcpp
  ${sensor_msgs_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
  ${GSTREAMER_APP_LIBRARIES}
  ${OPENCV_LIBS}
  cv_bridge::cv_bridge
)

install(TARGETS gstreamer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
