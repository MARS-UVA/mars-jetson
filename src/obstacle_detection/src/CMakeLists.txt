cmake_minimum_required(VERSION 3.8)
project(realsense_capture)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Windows-specific configurations
if(MSVC)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
    # Disable specific warnings
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4819 /wd4251 /wd4244 /wd4267")
endif()

set(REALSENSE_SOURCE_DIR "librealsense2-pkg/librealsense-win")
set(REALSENSE_INCLUDE_DIR "${REALSENSE_SOURCE_DIR}/include")

set(CMAKE_MODULE_PATH "${REALSENSE_SOURCE_DIR}/CMake" ${CMAKE_MODULE_PATH})

# Add RealSense library subdirectory
add_subdirectory(${REALSENSE_SOURCE_DIR})

add_executable(${PROJECT_NAME} realsense_ply.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${REALSENSE_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    realsense2
)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")