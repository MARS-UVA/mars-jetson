cmake_minimum_required(VERSION 3.8)
project(obstacle_detection)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Windows/MSVC config
if(MSVC)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")
    foreach(flag_var
        CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
        CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO)
        if(${flag_var} MATCHES "/MT")
            string(REGEX REPLACE "/MT" "/MD" ${flag_var} "${${flag_var}}")
        endif()
    endforeach()
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

# Paths for RealSense
set(REALSENSE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/realsense-source/librealsense-2.56.3")
set(REALSENSE_INCLUDE_DIR "${REALSENSE_SOURCE_DIR}/include")

# Disable RealSense extras
set(BUILD_EXAMPLES OFF CACHE BOOL "Disable RealSense examples")
set(BUILD_GRAPHICAL_EXAMPLES OFF CACHE BOOL "Disable RealSense graphical examples")
set(BUILD_TOOLS OFF CACHE BOOL "Disable RealSense tools")
set(BUILD_GLFW OFF CACHE BOOL "Disable GLFW/OpenGL components")
set(BUILD_WITH_STATIC_CRT OFF CACHE BOOL "Build with static C runtime" FORCE)
set(BUILD_SHARED_LIBS ON CACHE BOOL "Build shared libraries" FORCE)

# Required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)

# Add RealSense source
add_subdirectory(${REALSENSE_SOURCE_DIR})

# Create executable
add_executable(${PROJECT_NAME}
    src/obstacle_detect_node.cpp
    src/realsense_capture.cpp
    src/models/obstacle_clustering_tree.cpp
    src/gradient_map.cpp
    src/local_path_planner_graph.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
    ${REALSENSE_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE
    realsense2
    ${OpenCV_LIBS}
    rclcpp
)

# Install target
install(TARGETS
    ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

# Testing (optional)
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_cpplint_FOUND TRUE)
    set(ament_cmake_copyright_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
