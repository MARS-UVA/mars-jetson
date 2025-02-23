#ifndef SERVER_H
#define SERVER_H

#include <sys/socket.h>
#include <netinet/in.h>
#include "main.hpp"

struct {
    char[2000] client_message
    bool global_flag
  } ThreadInfo;

int create_server();

#endif