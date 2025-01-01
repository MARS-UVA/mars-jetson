#include <string.h>
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

#define PORT 8080
#define CHUNK_SIZE 1400

void usleep_simulation(unsigned int microseconds)
{
    volatile unsigned long long i;
    for (i = 0; i < (unsigned long long)microseconds * 1000; i++)
    {
        // Do nothing, just burn CPU cycles
    }
}

void create_sending_client(char *ip, unsigned char *data, size_t data_size)
{
    int socket_desc;
    struct sockaddr_in server_addr;
    char server_message[2000], client_message[2000];
    socklen_t server_struct_length = sizeof(server_addr);
    int total_chunks, start, end;

    // Clean buffers:
    // memset(server_message, '\0', sizeof(server_message));
    // memset(client_message, '\0', sizeof(client_message));

    // Create socket:
    socket_desc = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (socket_desc < 0)
    {
        printf("Error while creating socket\n");
        return;
    }
    printf("Socket created successfully\n");

    // Set port and IP:
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);
    server_addr.sin_addr.s_addr = inet_addr(ip);

    // Calculate number of chunks to send
    total_chunks = (data_size + CHUNK_SIZE - 1) / CHUNK_SIZE;
    // Send data in chunks
    for (int i = 0; i < total_chunks; i++)
    {
        usleep(1000);
        start = i * CHUNK_SIZE;
        end = (start + CHUNK_SIZE > data_size) ? data_size : start + CHUNK_SIZE;
        // add 3 bytes for metadata
        unsigned char packet[CHUNK_SIZE + 3];
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
        struct sockaddr *destination_addr_ptr = (struct sockaddr *)&server_addr;
        if (sendto(socket_desc, packet, end - start + 3, 0, destination_addr_ptr, server_struct_length) < 0)
        {
            printf("Unable to send message\n");
            return;
        }
    }

    /*
        // Send the message to server:
        if(sendto(socket_desc, data, sizeof(data), 0,
             (struct sockaddr*)&server_addr, server_struct_length) < 0){
            printf("Unable to send message\n");
            return;
        }
    */

    // Receive the server's response:
    // if (recvfrom(socket_desc, server_message, sizeof(server_message), 0,
    //              (struct sockaddr *)&server_addr, &server_struct_length) < 0)
    // {
    //     printf("Error while receiving server's msg\n");
    //     return;
    // }

    // printf("Server's response: %s\n", server_message);

    // Close the socket:
    close(socket_desc);
}

/*int main(int argc, char *argv[]) {
    client(argv[1], "hello");
}*/
