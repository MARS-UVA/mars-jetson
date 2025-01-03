#include <string.h>
#include <iostream>
#include <unistd.h>
#include <cstring>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "client.hpp"
// #include "frame_sender.hpp"

#ifdef _WIN32
void usleep_simulation(unsigned int microseconds);
#define usleep usleep_simulation
#include <winsock2.h>
#include <ws2tcpip.h>
#include <iphlpapi.h>
#include <stdio.h>
#pragma comment(lib, "Ws2_32.lib")
#define close closesocket
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

ConnectionHeaders create_connection_headers(const char *control_station_ip)
{
    /* Create new client socket to send frame: */
    int client_socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (client_socket_fd < 0)
    {
        throw std::runtime_error("Error while creating socket");
    }

    /* Set port and IP for control station laptop: */
    struct sockaddr_in control_station_addr;
    socklen_t contol_station_struct_len = sizeof(control_station_addr);
    control_station_addr.sin_family = AF_INET;
    control_station_addr.sin_port = htons(PORT);
    control_station_addr.sin_addr.s_addr = inet_addr(control_station_ip);

    ConnectionHeaders connection_headers = {client_socket_fd, control_station_addr};
    return connection_headers;
}

void client_send(const char *control_station_ip, unsigned char *data, size_t data_size)
{
    ConnectionHeaders connection_headers = create_connection_headers(control_station_ip);

    // Send the message to server:
    size_t sent_bytes = 0;
    while (sent_bytes < data_size)
    {
        size_t bytes_to_send = std::min(CHUNK_SIZE, (int)(data_size - sent_bytes));
        ssize_t transmission_result = sendto(connection_headers.client_socket_fd,
                                             data + sent_bytes,
                                             bytes_to_send, 0,
                                             (struct sockaddr *)&(connection_headers.control_station_addr),
                                             sizeof(connection_headers.control_station_addr));
        if (transmission_result < 0)
        {
            throw std::runtime_error("Unable to send message");
            return;
        }
        sent_bytes += transmission_result;
    }
}

void client_send(const char *control_station_ip, cv::Mat &image)
{
    ConnectionHeaders connection_headers = create_connection_headers(control_station_ip);
}
