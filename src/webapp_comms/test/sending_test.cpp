#include <catch2/catch_all.hpp>
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

int create_test_server()
{
    int server_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (server_socket_fd < 0)
    {
        throw std::runtime_error("Error creating server socket");
    }
    struct sockaddr_in server_address;
    memset(&server_address, '\0', sizeof(server_address));
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(2000);
    server_address.sin_addr.s_addr = INADDR_ANY;

    int reuse_option = 1;
    if (setsockopt(server_socket_fd, SOL_SOCKET, SO_REUSEADDR, (const char *)&reuse_option, sizeof(int)) < 0)
    {
        throw std::runtime_error("Error setting socket options");
    }

    struct sockaddr *server_address_ptr = (struct sockaddr *)&server_address;
    if (bind(server_socket_fd, server_address_ptr, sizeof(server_address)) < 0)
    {
        std::cout << "Bind error number: " << errno << std::endl;
        throw std::runtime_error("Error binding server socket");
    }
    return server_socket_fd;
}

TEST_CASE("Test client sending small data to a server [client_udp.h]")
{
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
        REQUIRE(((DataHeader *)buffer)->packetNum == 0);
        REQUIRE(((DataHeader *)buffer)->fragment_size == data_size);
        REQUIRE(((DataHeader *)buffer)->crc == crc32bit((char *)data, data_size));
        break;
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Elapsed time of [Small Data] test: " << elapsed_seconds.count() << "s\n";

    client_thread.join();
    close(server_socket_fd);
}

TEST_CASE("Test client sending large data to a server [client_udp.h]")
{
    int server_socket_fd = create_test_server();
    REQUIRE(server_socket_fd > 0);

    auto start = std::chrono::high_resolution_clock::now();

    // Set non-blocking mode to avoid a situation where the client thread has executed before the server is ready to recieve/recvfrom is called
    int flags = fcntl(server_socket_fd, F_GETFL, 0);
    fcntl(server_socket_fd, F_SETFL, flags | O_NONBLOCK);

    std::vector<unsigned char> data(10000);
    for (size_t i = 0; i < data.size(); i++)
    {
        data[i] = (unsigned char)(i % 256);
    }
    const char *ip = "127.0.0.1";

    // Transmission should be within 1 second - 1 second timeouyt for socket
    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(server_socket_fd, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(tv));

    // Increase socket buffer sizes to recieve larger amounts of data - this is like the receive window size in TCP
    int rcvbuf = 262144; // 256KB of buffer space to receivq data
    setsockopt(server_socket_fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    std::thread client_thread([&]()
                              { client_send(ip, data.data(), data.size()); });

    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    char buffer[1410];
    std::vector<unsigned char> received_data;

    auto start_time = std::chrono::steady_clock::now();
    while (received_data.size() < data.size())
    {
        ssize_t num_bytes = recvfrom(server_socket_fd,
                                     buffer, sizeof(buffer), 0,
                                     (struct sockaddr *)&client_addr,
                                     &client_len);

        if (num_bytes < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(1))
                {
                    throw std::runtime_error("Timeout of 1 second while waiting for data");
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            std::cerr << "recvfrom error: " << strerror(errno) << std::endl;
            break;
        }
        uint32_t crc = crc32bit(buffer + HEADER_SIZE, num_bytes - HEADER_SIZE);
        REQUIRE(crc == ((DataHeader *)buffer)->crc);
        REQUIRE(((DataHeader *)buffer)->fragment_size == num_bytes - HEADER_SIZE);
        char *payloadStart = buffer + HEADER_SIZE;
        received_data.insert(received_data.end(), payloadStart, payloadStart + ((DataHeader *)buffer)->fragment_size);
    }

    client_thread.join();

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Elapsed time of [Large Data] test: " << elapsed_seconds.count() << "s\n";

    REQUIRE(received_data.size() == data.size());
    REQUIRE(memcmp(received_data.data(), data.data(), data.size()) == 0);
    close(server_socket_fd);
}

TEST_CASE("Test client sending image to a server [client_udp.h]")
{
    int server_socket_fd = create_test_server();
    REQUIRE(server_socket_fd > 0);

    auto start = std::chrono::high_resolution_clock::now();

    // Set non-blocking mode to avoid a situation where the client thread has executed before the server is ready to recieve/recvfrom is called
    int flags = fcntl(server_socket_fd, F_GETFL, 0);
    fcntl(server_socket_fd, F_SETFL, flags | O_NONBLOCK);

    cv::Mat imageMatrix = cv::imread("../assets/test1.png", cv::IMREAD_COLOR);
    std::vector<unsigned char> imgBuffer;
    std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 90};
    cv::imencode(".jpg", imageMatrix, imgBuffer, compression_params);

    const char *ip = "127.0.0.1";

    // Transmission should be within 1 second - 1 second timeouyt for socket
    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(server_socket_fd, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(tv));

    // Increase socket buffer sizes to recieve larger amounts of data - this is like the receive window size in TCP
    int rcvbuf = 262144; // 256KB of buffer space to receivq data
    setsockopt(server_socket_fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    std::thread client_thread([&]()
                              { client_send(ip, imageMatrix); });

    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    char buffer[1410];
    std::vector<unsigned char> received_data;

    auto start_time = std::chrono::steady_clock::now();
    while (received_data.size() < imgBuffer.size())
    {
        ssize_t num_bytes = recvfrom(server_socket_fd,
                                     buffer, sizeof(buffer), 0,
                                     (struct sockaddr *)&client_addr,
                                     &client_len);

        if (num_bytes < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(1))
                {
                    throw std::runtime_error("Timeout of 1 second while waiting for data");
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            std::cerr << "recvfrom error: " << strerror(errno) << std::endl;
            break;
        }
        uint32_t crc = crc32bit(buffer + HEADER_SIZE, (size_t)((DataHeader *)buffer)->fragment_size);
        REQUIRE(crc == ((DataHeader *)buffer)->crc);
        REQUIRE(((DataHeader *)buffer)->fragment_size == num_bytes - HEADER_SIZE);
        char *payloadStart = buffer + HEADER_SIZE;
        received_data.insert(received_data.end(), payloadStart, payloadStart + ((DataHeader *)buffer)->fragment_size);
    }

    client_thread.join();

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Elapsed time of [Image Data] test: " << elapsed_seconds.count() << "s\n";

    REQUIRE(received_data.size() == imgBuffer.size());
    REQUIRE(memcmp(received_data.data(), imgBuffer.data(), imgBuffer.size()) == 0);
    cv::Mat receivedImage = cv::imdecode(received_data, cv::IMREAD_COLOR);
    cv::imshow("Received Image", receivedImage);
    REQUIRE(receivedImage.rows == imageMatrix.rows);
    REQUIRE(receivedImage.cols == imageMatrix.cols);
    cv::waitKey(0);
    close(server_socket_fd);
}
