#ifndef CLIENT_H
#define CLIENT_H

#define PORT 2000
#define CHUNK_SIZE 1400

#include <sys/socket.h>
#include <netinet/in.h>
#include "main.hpp"

struct connection_headers
{
    int client_socket_fd;
    struct sockaddr_in control_station_addr;
} typedef ConnectionHeaders;
ConnectionHeaders create_connection_headers(const char *control_station_ip);

struct DataHeader
{
    int sequence;
    uint16_t fragment_size;
    uint32_t crc;
} __attribute__((packed));
#define HEADER_SIZE sizeof(DataHeader)
typedef struct DataHeader DataHeader;

uint32_t crc32bit(const char *data, size_t data_size);
void client_send(const char *control_station_ip, unsigned char *data, size_t data_size);
void client_send(const char *control_station_ip, cv::Mat &image);

#endif
