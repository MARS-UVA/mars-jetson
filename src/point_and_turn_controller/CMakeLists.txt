cmake_minimum_required(VERSION 3.8)
project(point_and_turn_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ROS-free core: path simplifier and state machine. Static so the plugin
# carries it with no extra install step; position-independent because it is
# linked into the plugin's shared library.
add_library(point_and_turn_core STATIC src/core.cpp)
target_include_directories(point_and_turn_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
set_target_properties(point_and_turn_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# The plugin: everything that needs ROS.
add_library(${PROJECT_NAME} SHARED src/point_and_turn_controller.cpp)
target_link_libraries(${PROJECT_NAME} point_and_turn_core)
ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  tf2_ros
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# Registers the class with pluginlib under nav2_core, which is how the
# controller server finds "point_and_turn_controller::PointAndTurnController".
pluginlib_export_plugin_description_file(nav2_core point_and_turn_controller.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Core only: no ROS graph, no costmap, so they run in milliseconds.
  ament_add_gtest(test_simplifier test/test_simplifier.cpp)
  target_link_libraries(test_simplifier point_and_turn_core)
  ament_add_gtest(test_core test/test_core.cpp)
  target_link_libraries(test_core point_and_turn_core)
endif()

ament_package()
