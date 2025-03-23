#ifndef SERVER_H
#define SERVER_H

#include <sys/socket.h>
#include <netinet/in.h>
#include "main.hpp"

struct ThreadInfo {
    char client_message[100000];
    bool flag;
};

int create_server(ThreadInfo* info);

#endif