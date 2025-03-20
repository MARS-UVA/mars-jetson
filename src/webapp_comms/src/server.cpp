#include "./server.hpp"
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netinet/in.h>
#include <thread>
#include "./client.hpp"
#include <cstdio>
#include <stdexcept>

#define PORT 8080

int create_server(ThreadInfo *info)
{
    int socket_desc;
    struct sockaddr_in server_addr, client_addr;
    memset(&server_addr, '\0', sizeof(server_addr));
    char server_message[2000], client_message[2000];
    socklen_t client_struct_length = sizeof(client_addr);

    std::string cmd = "hostname -I | awk '{print $1}";
    std::vector<char> inet_buffer(128);
    std::string localIp;
    FILE* pipe = popen(cmd, "r");
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(inet_buffer.data(), inet_buffer.size(), pipe) != nullptr) {
        localIp += inet_buffer.data();
    }
    pclose(pipe);

    if (!localIp.empty() && localIp.back() == '\n') {
        localIp.pop_back();
    }

    std::cout << localIp << std::endl;

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
    server_addr.sin_port = htons(8080);
    server_addr.sin_addr.s_addr = inet_addr("172.25.153.79");

    // Bind to the set port and IP:
    int reuse_option = 1;
    if (setsockopt(socket_desc, SOL_SOCKET, SO_REUSEADDR, (const char *)&reuse_option, sizeof(int)) < 0)
    {
        throw std::runtime_error("Error setting socket options");
    }

    struct sockaddr *server_addr_ptr = (struct sockaddr *)&server_addr;
    if (bind(socket_desc, server_addr_ptr, sizeof(server_addr)) < 0)
    {
        std::cout << "Bind error number: " << errno << std::endl;
        throw std::runtime_error("Error binding server socket");
    }
    printf("Done with binding\n");

    printf("Listening for incoming messages...\n\n");

    std::vector<uint16_t> pkts_to_retry;

    socklen_t client_len = sizeof(client_addr);
    char buffer[1410];
    memset(buffer, '\0', 1410);
    std::vector<unsigned char> received_data;
    
    ssize_t num_bytes = recvfrom(socket_desc,
                                     buffer, sizeof(buffer), 0,
                                     (struct sockaddr *)&client_addr,
                                     &client_len);

    printf("Recvieved message");

    if (num_bytes < 0)
    {
        std::cerr << "recvfrom error: " << strerror(errno) << std::endl;
    }
    uint32_t crc = crc32bit(buffer + HEADER_SIZE, num_bytes - HEADER_SIZE);
    if(crc != ((DataHeader *)buffer)->crc || ((DataHeader *)buffer)->fragment_size != num_bytes - HEADER_SIZE){
        // Handle this later
    }
    char *payloadStart = buffer + HEADER_SIZE;
    received_data.insert(received_data.end(), payloadStart, payloadStart + ((DataHeader *)buffer)->fragment_size);

    received_data.push_back('\0');
    memset(info->client_message, '\0', 100000);
    memcpy(info->client_message, received_data.data(), received_data.size());

    info->flag = true;

    std::cout << buffer << std::endl;

    // Close the socket:
    close(socket_desc);

    return 0;
}
