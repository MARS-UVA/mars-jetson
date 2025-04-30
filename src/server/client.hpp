#ifndef CLIENT_H
#define CLIENT_H

#include <sys/socket.h>
#include <netinet/in.h>
#include "main.hpp"

struct connection_headers
{
    int client_socket_fd;
    struct sockaddr_in control_station_addr;
} typedef ConnectionHeaders;
ConnectionHeaders create_connection_headers(int port);

struct DataHeader
{
    uint16_t packetNum;
    uint16_t totalPackets;
    uint16_t fragment_size;
    uint32_t crc;
} __attribute__((packed));
#define HEADER_SIZE sizeof(DataHeader)
typedef struct DataHeader DataHeader;

uint32_t crc32bit(const char *data, size_t data_size);
void client_send(unsigned char *data, size_t data_size, int port);
void client_send(cv::Mat &image, int port);

#endif
