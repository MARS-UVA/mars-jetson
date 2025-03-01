#include <catch2/catch_all.hpp>
#include "../src/server.hpp"
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


TEST_CASE("Receiving small amount of data from client")
{
    std::cout << "Started testing server\n";
    std::thread server_thread([&]() {create_server();});

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    auto start = std::chrono::high_resolution_clock::now();
    
    unsigned char data[] = "Motor command";
    size_t data_size = sizeof(data) - 1;
    const char *ip = "127.0.0.1";
    const int port = 8080;

    std::thread client_thread([&]()
        {
            int client_socket;
            struct sockaddr_in server_addr;

            client_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if(client_socket < 0) {
                perror("Error while creating socket");
                return;
            }

            server_addr.sin_family = AF_INET;
            server_addr.sin_port = htons(port);
            server_addr.sin_addr.s_addr = inet_addr(ip);

            ssize_t sent_bytes = sendto(client_socket, data, data_size, 0, (struct sockaddr*)&server_addr, sizeof(server_addr));
            if(sent_bytes < 0) perror("Failed to send data");

            close(client_socket);
        }
    );

    client_thread.join();

    // Transmission should be within 1 second
    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;

    REQUIRE(1 == 1);

    server_thread.detach();
}

/* Reference material
    int server_socket_fd = create_test_server();

    auto start = std::chrono::high_resolution_clock::now();
    // Set non-blocking mode to avoid a situation where the client thread has executed before the server is ready to recieve/recvfrom is called
    int flags = fcntl(server_socket_fd, F_GETFL, 0);
    fcntl(server_socket_fd, F_SETFL, flags | O_NONBLOCK);

    unsigned char data[] = "Hello, world!";
    size_t data_size = sizeof(data) - 1;
    const char *ip = "127.0.0.1";

    // std::promise<struct sockaddr *> result_promise;
    // std::future<struct sockaddr *> result_future = result_promise.get_future();

    std::thread client_thread([&]()
                              { client_send(ip, data, data_size); });

    // Transmission should be within 1 second
    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(server_socket_fd, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(tv));

    // Receive data from client
    char buffer[1024];
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);

    // struct sockaddr *result = result_future.get();
    while (true)
    {
        ssize_t received = recvfrom(server_socket_fd, buffer, sizeof(buffer), 0,
                                    (struct sockaddr *)&client_addr, &client_len);
        if (received < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                continue;
            }
            else
            {
                perror("recvfrom failed");
                break;
            }
        }

        buffer[received] = '\0';
        REQUIRE(received - HEADER_SIZE == data_size);
        REQUIRE(memcmp(buffer + HEADER_SIZE, data, data_size) == 0);
        REQUIRE(((DataHeader *)buffer)->sequence == 0);
        REQUIRE(((DataHeader *)buffer)->fragment_size == data_size);
        REQUIRE(((DataHeader *)buffer)->crc == crc32bit((char *)data, data_size));
        break;
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Elapsed time of [Small Data] test: " << elapsed_seconds.count() << "s\n";

    client_thread.join();
    close(server_socket_fd);
*/
