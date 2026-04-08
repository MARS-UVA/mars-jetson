#ifndef CLIENT_H
#define CLIENT_H

#include "main.hpp"

struct connection_headers
{
    int client_socket_fd;
    struct sockaddr_in control_station_addr;
} typedef ConnectionHeaders;

//const char* CONTROL_STATION_IP_FOR_CLIENT = "192.168.0.100";
//const char* CONTROL_STATION_IP = "192.168.0.200";
ConnectionHeaders create_connection_headers(int port);

struct DataHeader
{
    uint16_t packetToSend;
    uint16_t totalPacketsToSend;
    uint16_t fragmentSize;
    uint32_t crc;
} __attribute__((packed));
#define HEADER_SIZE sizeof(DataHeader)
typedef struct DataHeader DataHeader;

uint32_t crc32bit(const char *data, size_t data_size);

/*client_send: sends data
Parameters:
*data: the data to be sent
size_t: the size of the data
server_port: the destination port, this will be the port of the server on the control station*/
void client_send(unsigned char *data, size_t data_size, int server_port);

#endif
