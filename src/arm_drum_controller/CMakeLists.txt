cmake_minimum_required(VERSION 3.8)
project(arm_drum_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(robot_control_msgs REQUIRED)

add_library(arm_drum_controller SHARED src/arm_drum_controller.cpp)
target_link_libraries(arm_drum_controller PRIVATE
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  realtime_tools::realtime_tools
  robot_control_msgs::robot_control_msgs
)

# Export the plugin description file
pluginlib_export_plugin_description_file(controller_interface plugins.xml)

install(TARGETS arm_drum_controller
  EXPORT export_arm_drum_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
