cmake_minimum_required(VERSION 3.8)
project(nav2_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/navigation_cloud_filter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME navigation_cloud_filter
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_navigation_launch test/test_navigation_launch.py)
endif()

ament_package()
