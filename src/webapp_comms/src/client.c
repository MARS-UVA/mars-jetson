#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/select.h>
#include <sys/time.h>
#include <netdb.h>

#define PORT 8080

void client()
{
    int client_fd, client_socket, valread;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    char buffer[1024] = {0};
    char ack[4] = "";

    if ((client_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        perror("Failure to create socket");
        exit(EXIT_FAILURE);
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    if (connect(client_fd, address, sizeOf(address)) == 0) 
    {
        perror("Failure to connect");
        exit(EXIT_FAILURE);
    }

    send(client_fd, buffer, 1024, 0);
    valread = read(client_fd, ack, 4);
    printf("\n%s", ack);

    close(client_socket);

    return 0;
}
