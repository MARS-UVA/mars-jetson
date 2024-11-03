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

void client(char* ip)
{
    int client_fd, client_socket, valread;
    struct sockaddr_in address;
    int addrlen = sizeof(address);
    char buffer[1024];
    char ack[1000];

    if ((client_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
    {
        perror("Failure to create socket");
        exit(EXIT_FAILURE);
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr(ip);
    address.sin_port = htons(PORT);

    if (connect(client_fd, (struct sockaddr*)&address, sizeof(address)) != 0) 
    {
        perror("Failure to connect");
        exit(EXIT_FAILURE);
    }
     printf("Connected to server at %s:%d\n", ip, PORT);

    strncpy(buffer, "hello", 5);
    if (send(client_fd, buffer, strlen(buffer), 0) == -1) {
            perror("Send failed");
    }

    valread = read(client_fd, ack, 1000);
    if (valread < 0) {
            perror("Read failed");
        } else if (valread == 0) {
            printf("Server closed the connection\n");
        }
    ack[valread] = '\0';
    printf("%s\n", ack);
    close(client_fd);
    return;
}

int main(int argc , char *argv[]){
    client(argv[1]);
}

