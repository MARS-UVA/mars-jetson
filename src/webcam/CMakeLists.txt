cmake_minimum_required(VERSION 3.8)
project(webcam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include)

add_executable(webcam
  src/webcam.cpp
  ../server/server.cpp
  ../server/client.cpp
  ../server/frame_sender.cpp
)

target_link_libraries(webcam ${OpenCV_LIBS})
target_include_directories(webcam PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(webcam 
  rclcpp 
  std_msgs 
  sensor_msgs 
  cv_bridge 
  OpenCV 
)

install(TARGETS
  webcam
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

# ament_python_install_package(${PROJECT_NAME})
# install(PROGRAMS
#   scripts/webcam_publisher.py
#   scripts/display.py
#   DESTINATION lib/${PROJECT_NAME})

ament_package()
