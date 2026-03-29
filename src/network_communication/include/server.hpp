#ifndef SERVER_H
#define SERVER_H

#include <sys/socket.h>
#include <netinet/in.h>
#include "main.hpp"

struct ThreadInfo {
    char client_message[100000];
    bool controller_flag;
    bool auto_flag;
    int8_t robot_action;
};

int create_server(ThreadInfo* info);

#endif