cmake_minimum_required(VERSION 3.8)
project(webapp_comms)

find_package(Catch2 3 REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(test
    sending_test.cpp
    ../src/client.cpp
)

target_link_libraries(test PRIVATE Catch2::Catch2WithMain ${OpenCV_LIBS})

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")