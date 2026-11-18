cmake_minimum_required(VERSION 3.8)
project(robot_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/navigation_cloud_filter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME navigation_cloud_filter
)

install(PROGRAMS scripts/arena_markers_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME arena_markers_node
)
install(FILES scripts/arena_configs.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_navigation_launch test/test_navigation_launch.py)
endif()

ament_package()
