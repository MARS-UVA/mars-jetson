cmake_minimum_required(VERSION 3.8)
project(webapp_comms)

find_package(Catch2 3 REQUIRED)

add_executable(test
    sending_test.cpp
    ../src/client_udp.c
)

target_link_libraries(test PRIVATE Catch2::Catch2WithMain)