cmake_minimum_required(VERSION 3.8)
project(realsense_capture)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
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
add_executable(${PROJECT_NAME} realsense_ply.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${REALSENSE_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    realsense2
)