#ifndef SERVER_H
#define SERVER_H

#include <cstdio>
#include <stdexcept>
#include <csignal>

#include "main.hpp"
#include "client.hpp"

/*Port for receiving*/
#define PORT 8080

struct ThreadInfo {
    char client_message[100000];
    bool controller_flag;
    bool auto_flag;
    int8_t robot_action;
};

//Creates the server in a thread
int create_server(ThreadInfo* info);

#endif