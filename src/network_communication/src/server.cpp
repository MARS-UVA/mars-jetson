#include <unistd.h>
#include <netinet/in.h>
#include <string.h>
#include <thread>
#include <sys/socket.h>
#include <arpa/inet.h>
#include "main.hpp"
#include <stdexcept>

int create_server (ThreadInfo *info){
    struct sockaddr_in server_addr, client_addr;
    memset(&server_addr, '\0', sizeof(server_addr));
    char server_message[2000], client_message[2000];
    socklen_t client_struct_length = sizeof(client_addr);

    int socket_desc = socket (AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(10000);
    server_addr.sin_addr.s_addr = inet_addr("0.0.0.0");

    // Bind to the set port and IP:
    int reuse_option = 1;

    if (setsockopt(socket_desc, SOL_SOCKET, SO_REUSEADDR, (const char *)&reuse_option, sizeof(int)) < 0) {
        throw std::runtime_error("Error setting socket options");
    }

    struct sockaddr *server_addr_ptr = (struct sockaddr *)&server_addr;
    socklen_t client_len = sizeof(client_addr);
    bind (socket_desc, server_addr_ptr, sizeof(server_addr));

    
    while (true){
        recvfrom (socket_desc, info->client_message, sizeof(info->client_message),0, (struct sockaddr *)&client_addr, &client_len);
        info->flag = true;
    }


   close (socket_desc);
    return 0;

}





