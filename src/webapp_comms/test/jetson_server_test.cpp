#include <catch2/catch_all.hpp>
#include "../src/server.hpp"
#include "../src/client.hpp"
#include <vector>
#include <thread>
#include <future>
#include <chrono>
#include <iostream>
#include <cstring>
#include <fcntl.h>
#include <chrono>
#include <opencv2/opencv.hpp>

#include "../../platform_compat.h"

TEST_CASE("Test server receiving small data")
{
    ThreadInfo info;
    info.flag = false;
    memset(info.client_message, '\0', sizeof(info.client_message));

    std::thread socket(create_server, &info);

    unsigned char data[] = "Hello, world!";
    size_t data_size = sizeof(data) - 1;
    const char *ip = "127.0.0.1";

    usleep(1000);
    auto start = std::chrono::high_resolution_clock::now();

    client_send(ip, data, data_size, 8080);

    while(info.flag == false); {}
    REQUIRE(info.flag == true);
    REQUIRE(memcmp(info.client_message, data, data_size) == 0);




    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Elapsed time of [Small Data] test: " << elapsed_seconds.count() << "s\n";

    socket.join();
}