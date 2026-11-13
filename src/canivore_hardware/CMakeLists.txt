cmake_minimum_required(VERSION 3.8)
project(canivore_hardware)

# Phoenix 6 headers use C++20 features (std::span, etc.)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(serial_msgs REQUIRED)
# Installed by CTRE's "phoenix6" APT package (config lives in /usr/lib/phoenix6/cmake).
find_package(phoenix6 REQUIRED PATHS /usr/lib/phoenix6/cmake)

add_library(${PROJECT_NAME} SHARED
  src/canivore_hardware.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} phoenix6)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  serial_msgs
)

# Phoenix installs its .so files in /usr/lib/phoenix6, outside the default library path.
# Without this RPATH the plugin builds but fails to load at runtime.
get_filename_component(PHOENIX6_LIBRARY_DIR "${phoenix6_DIR}/.." ABSOLUTE)
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "${PHOENIX6_LIBRARY_DIR}")

pluginlib_export_plugin_description_file(hardware_interface canivore_hardware.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle serial_msgs)
ament_package()
