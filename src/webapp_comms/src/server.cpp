#include "./server.hpp"
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netinet/in.h>
#include <thread>

#define PORT 8080

int create_server(ThreadInfo *info)
{
    int socket_desc;
    struct sockaddr_in server_addr, client_addr;
    memset(&server_address, '\0', sizeof(server_address));
    char server_message[2000], client_message[2000];
    socklen_t client_struct_length = sizeof(client_addr);

    // Clean buffers:
    // memset(server_message, '\0', sizeof(server_message));
    // memset(client_message, '\0', sizeof(client_message));

    // Create UDP socket:
    socket_desc = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (socket_desc < 0)
    {
        printf("Error while creating socket\n");
        return -1;
    }
    printf("Socket created successfully\n");

    // Set port and IP:
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);
    server_addr.sin_addr.s_addr = inet_addr("127.0.0.1");

    // Bind to the set port and IP:
    int reuse_option = 1;
    if (setsockopt(socket_desc, SOL_SOCKET, SO_REUSEADDR, (const char *)&reuse_option, sizeof(int)) < 0)
    {
        throw std::runtime_error("Error setting socket options");
    }

    struct sockaddr *server_address_ptr = (struct sockaddr *)&server_address;
    if (bind(socket_desc, server_address_ptr, sizeof(server_address)) < 0)
    {
        std::cout << "Bind error number: " << errno << std::endl;
        throw std::runtime_error("Error binding server socket");
    }
    printf("Done with binding\n");

    printf("Listening for incoming messages...\n\n");

    // while (1)
    // {
    //     // Receive client's message:
    //     if (recvfrom(socket_desc, client_message, sizeof(client_message), 0,
    //                  (struct sockaddr *)&client_addr, &client_struct_length) < 0)
    //     {
    //         printf("Couldn't receive\n");
    //         return -1;
    //     }
    //     printf("Received message from IP: %s and port: %i\n",
    //            inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));

    //     printf("Msg from client: %s\n", client_message);

    //     info->flag = true;
    //     memcpy(info->client_message, client_message, 2000);

    //     // if (sendto(socket_desc, server_message, strlen(server_message), 0,
    //     //            (struct sockaddr *)&client_addr, client_struct_length) < 0)
    //     // {
    //     //     printf("Can't send\n");
    //     //     return -1;
    //     // }
    // }

    std::vector<uint16_t> pkts_to_retry;

    socklen_t client_len = sizeof(client_addr);
    char buffer[1410];
    memset(buffer, '\0', 1410);
    std::vector<unsigned char> received_data;
    uint16_t packetNum = 0;
    while (received_data.size() < data.size())
    {
        ssize_t num_bytes = recvfrom(socket_desc,
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
        if(crc != ((DataHeader *)buffer)->crc || ((DataHeader *)buffer)->fragment_size != num_bytes - HEADER_SIZE){
            pkts_to_retry.push_back(packetNum);
        }
        char *payloadStart = buffer + HEADER_SIZE;
        received_data.insert(received_data.end(), payloadStart, payloadStart + ((DataHeader *)buffer)->fragment_size);
        packetNum++;
    }
    received_data.push_back('\0');
    memset(info->client_message, '\0', 100000);
    memcpy(info->client_message, received_data.data(), received_data.size());

    // Close the socket:
    close(socket_desc);

    return 0;
}