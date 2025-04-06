cmake_minimum_required(VERSION 3.8)
project(realsense_capture)

set(CMAKE_BUILD_TYPE Debug)

# For GCC/Clang
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Force MSVC to use dynamic runtime
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

set(REALSENSE_SOURCE_DIR "realsense-source/librealsense-2.56.3")
set(REALSENSE_INCLUDE_DIR "${REALSENSE_SOURCE_DIR}/include")

# Configure RealSense build options
set(BUILD_WITH_STATIC_CRT OFF CACHE BOOL "Build with static C runtime" FORCE)
set(BUILD_SHARED_LIBS ON CACHE BOOL "Build shared libraries" FORCE)

set(CMAKE_MODULE_PATH "${REALSENSE_SOURCE_DIR}/CMake" ${CMAKE_MODULE_PATH})

# Add RealSense library subdirectory
add_subdirectory(${REALSENSE_SOURCE_DIR})


# Create executable
add_executable(${PROJECT_NAME} startup.cpp realsense_capture.cpp models/obstacle_clustering_tree.cpp gradient_map.cpp local_path_planner_graph.cpp)
add_executable(gradientMapTest ../test/gradient_map_test.cpp extract_capture.cpp realsense_capture.cpp gradient_map.cpp models/obstacle_clustering_tree.cpp)
add_executable(obstacleDetectionTest ../test/obstacle_detection_test.cpp extract_capture.cpp realsense_capture.cpp gradient_map.cpp models/obstacle_clustering_tree.cpp)
# add_executable(pcTreeSimpleTests EXCLUDE_FROM_ALL models/pc_adacency_tree.cpp ../test/pc_tree_simple_tests.cpp)
# add_executable(pcTreeQuadrantTest models/pc_adacency_tree.cpp ../test/pc_tree_quadrant_test.cpp realsense_capture.cpp)
# add_executable(pcTreeGradientTest models/pc_adacency_tree.cpp models/pc_adacency_tree_gradientify.cpp ../test/pc_tree_gradient_test.cpp realsense_capture.cpp)

foreach(target ${PROJECT_NAME} gradientMapTest obstacleDetectionTest)
    target_include_directories(${target} PRIVATE
        ${REALSENSE_INCLUDE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
endforeach()

foreach(target ${PROJECT_NAME} gradientMapTest obstacleDetectionTest)
    target_link_libraries(${target} PRIVATE
        realsense2
    )
endforeach()