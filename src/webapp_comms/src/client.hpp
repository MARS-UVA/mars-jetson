#ifndef CLIENT_H
#define CLIENT_H

#define PORT 2000
#define CHUNK_SIZE 1400

#include <sys/socket.h>
#include <netinet/in.h>
#include <opencv2/opencv.hpp>

struct connection_headers
{
    int client_socket_fd;
    struct sockaddr_in control_station_addr;
} typedef ConnectionHeaders;

ConnectionHeaders create_connection_headers(const char *control_station_ip);

void client_send(const char *control_station_ip, unsigned char *data, size_t data_size);
void client_send(const char *control_station_ip, cv::Mat &image);

#endif
