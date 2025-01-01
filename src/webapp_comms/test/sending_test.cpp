#include <catch2/catch_all.hpp>
#include "../src/client_udp.h"
#include <vector>
#include <thread>
#include <chrono>
#include <iostream>
#include <cstring>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#define close closesocket // Map POSIX close to Windows closesocket
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

typedef int ssize_t;

int create_test_server()
{
    int server_socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (server_socket_fd < 0)
    {
        std::cerr << "Error creating server socket" << std::endl;
    }
    struct sockaddr_in server_address;
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(8080);
    server_address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    struct sockaddr *server_address_ptr = (struct sockaddr *)&server_address;
    if (bind(server_socket_fd, server_address_ptr, sizeof(server_address)) < 0)
    {
        std::cerr << "Error binding server socket" << std::endl;
    }
    return server_socket_fd;
}

TEST_CASE("Test client sending small data to a server [client_udp.h]")
{
    int server_socket_fd = create_test_server();
    REQUIRE(server_socket_fd > 0);
    unsigned char data[] = "Hello, world!";
    size_t data_size = sizeof(data) - 1;

    std::thread client_thread([&]()
                              { create_sending_client("127.0.0.1", data, data_size); });

    // Transmission should be within 2 seconds
    struct timeval tv;
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    setsockopt(server_socket_fd, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv);

    // Receive data from client
    char buffer[1024];
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    ssize_t received = recvfrom(server_socket_fd, buffer, sizeof(buffer), 0,
                                (struct sockaddr *)&client_addr, &client_len);
    REQUIRE(received - 3 == data_size);
    REQUIRE(memcmp(buffer + 3, data, data_size) == 0);
    client_thread.join();
    close(server_socket_fd);
}

TEST_CASE("Test client sending large data to a server [client_udp.h]")
{
    int server_socket_fd = create_test_server();
    REQUIRE(server_socket_fd > 0);

    std::vector<unsigned char> data(10000);
    for (int i = 0; i < 10000; i++)
    {
        data[i] = (unsigned char)i;
    }

    std::thread client_thread([&]()
                              { create_sending_client("127.0.0.1", data.data(), data.size()); });

    // Transmission should be within 2 seconds
    struct timeval tv;
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    setsockopt(server_socket_fd, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv);

    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    char buffer[1024];
    std::vector<unsigned char> received_data;

    while (true)
    {
        ssize_t num_bytes = recvfrom(server_socket_fd, buffer, sizeof(buffer), 0,
                                     (struct sockaddr *)&client_addr, &client_len);
        for (int i = 3; i < num_bytes; i++)
        {
            received_data.push_back(buffer[i]);
        }
        if (received_data.size() == 10000)
        {
            break;
        }
    }

    REQUIRE(received_data.size() == 10000);
    REQUIRE(memcmp(received_data.data(), data.data(), 10000) == 0);
    client_thread.join();
    close(server_socket_fd);
}
