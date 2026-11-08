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

# ROS-free core: path simplifier and state machine. Static so the plugin
# carries it with no extra install step; position-independent because it is
# linked into the plugin's shared library.
add_library(point_and_turn_core STATIC src/core.cpp)
target_include_directories(point_and_turn_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
set_target_properties(point_and_turn_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Core only: no ROS graph, no costmap, so they run in milliseconds.
  ament_add_gtest(test_simplifier test/test_simplifier.cpp)
  target_link_libraries(test_simplifier point_and_turn_core)
  ament_add_gtest(test_core test/test_core.cpp)
  target_link_libraries(test_core point_and_turn_core)
endif()

ament_package()
