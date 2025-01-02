#include <string.h>
#include <iostream>
#include <unistd.h>
#include <cstring>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "client_udp.h"

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

void usleep_simulation(unsigned int microseconds)
{
    volatile unsigned long long i;
    for (i = 0; i < (unsigned long long)microseconds * 1000; i++)
    {
        // Do nothing, just burn CPU cycles
    }
}

struct DataHeader
{
    int sequence;
    uint16_t fragment_size;
    uint32_t crc;
} __attribute__((packed));
#define HEADER_SIZE sizeof(DataHeader)

void send_frame(const char *control_station_ip, cv::Mat &image)
{
    char client_message[CHUNK_SIZE];

    int total_chunks, start, end;

    // Clean buffers:
    memset(client_message, '\0', sizeof(client_message));

    /* Frame compression and encoding */
    std::vector<uchar> buffer;
    std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 90};

    if (!cv::imencode(".jpg", image, buffer, compression_params))
    {
        std::cerr << "Image encoding failed!" << std::endl;
        return;
    }
    int buffer_size = buffer.size();
    total_chunks = (int)(buffer_size / CHUNK_SIZE) + (1 ? buffer_size % CHUNK_SIZE : 0);

    // Send data in chunks
    for (int i = 0; i < total_chunks; i++)
    {
        usleep(1000);
        start = i * CHUNK_SIZE;
        end = (start + DATA_MTU_SIZE > data_size) ? data_size : start + DATA_MTU_SIZE;
        // add 3 bytes for metadata
        unsigned char packet[DATA_MTU_SIZE + 3];
        // sequence #
        packet[0] = (unsigned char)i;
        // Total chunks count
        unsigned char total = (unsigned char)total_chunks;
        packet[1] = (total >> 8) & 0xFF;
        packet[2] = total & 0xFF;
        // Actual data
        memcpy(packet + 3, data + start, end - start);
        printf("%d of %d\n", i + 1, total_chunks);

        // send the chunks
        struct sockaddr *destination_addr_ptr = (struct sockaddr *)&control_station_addr;
        if (sendto(client_socket_fd, packet, end - start + 3, 0, destination_addr_ptr, contol_station_struct_len) < 0)
        {
            printf("Unable to send message\n");
            return;
        }
    }

    // Close the socket:
    close(client_socket_fd);
}
