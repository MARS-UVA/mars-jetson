/*Receives from the control station*/

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
#include <csignal>

/*Port for receiving*/
#define PORT 8080

/*
socket_desc is described below
socket_desc = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
*/
int socket_desc;

/*What does this do? What is signum?*/
void signal_handler(int signum)
{
    close(socket_desc);
    exit(signum);
}

/*Creates the server in a thread
ThreadInfo is for windows???????
https://github.com/MScholtes/ThreadInfo
I need to look more into ThreadInfo
*/
int create_server(ThreadInfo *info)
{
    /*Declare the addresses*/
    struct sockaddr_in server_addr, client_addr;
    memset(&server_addr, '\0', sizeof(server_addr));

    /*Guess: This sets maximum size*/
    char server_message[2000], client_message[2000];

    /*length of the client address struct in bytes*/
    socklen_t client_struct_length = sizeof(client_addr);

    /*
    const char* cmd = "hostname -I | awk '{print $1}\0";
    std::vector<char> inet_buffer(128);
    std::string localIp;
    FILE* pipe = popen(cmd, "r");
    if (!pipe) {
        throw std::runtime_error("failed!");
    }
    while (fgets(inet_buffer.data(), inet_buffer.size(), pipe) != nullptr) {
        localIp += inet_buffer.data();
    }
    pclose(pipe);

    if (!localIp.empty() && localIp.back() == '\n') {
        localIp.pop_back();
    }

    std::cout << localIp << std::endl;
    */
    // Clean buffers:
    // memset(server_message, '\0', sizeof(server_message));
    // memset(client_message, '\0', sizeof(client_message));

    /* Create UDP socket: */
    socket_desc = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    /*Ensure socket socket was actually created, otherwise exit*/
    if (socket_desc < 0)
    {
        printf("Error while creating socket\n");
        return -1;
    }
    // printf("Socket created successfully\n");

    /* Set port and IP: */
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(8080);
    /* 0.0.0.0 because it is local */
    server_addr.sin_addr.s_addr = inet_addr("0.0.0.0");

    /* Bind to the set port and IP: */
    int reuse_option = 1;
    /*
    SOL_SOCKET needed to actually manipulate the socket itself
    sets SO_REUSEADDR
    setsockopt returns -1 if there was an error
    */
    if (setsockopt(socket_desc, SOL_SOCKET, SO_REUSEADDR, (const char *)&reuse_option, sizeof(int)) < 0)
    {
        throw std::runtime_error("Error setting socket options");
    }

    /*
    Why must it be a pointer?
    What is this line doing?
    */
    struct sockaddr *server_addr_ptr = (struct sockaddr *)&server_addr;

    /* Actually creates the socket association, allows the code to interact with it*/
    if (bind(socket_desc, server_addr_ptr, sizeof(server_addr)) < 0)
    {
        std::cout << "Bind error number: " << errno << std::endl;
        throw std::runtime_error("Error binding server socket");
    }
    // printf("Done with binding\n");

    printf("Listening for incoming messages...\n\n");

    /* SIGINT: identification of the keyboard interupt
    signal_handler actually closes the sockets*/
    signal(SIGINT, signal_handler);

    /*Guess: Specifies what packets had corrupted data?*/
    std::vector<uint16_t> pkts_to_retry;

    /*client address*/
    socklen_t client_len = sizeof(client_addr);
    // printf("1");
    char buffer[1410];
    while (true)
    {
        /*Overwrites everything to ensure no data can impact what we read later*/
        memset(buffer, '\0', 1410);

        // printf("2");

        /*Stores the recieved data as a vector of characters*/
        std::vector<unsigned char> received_data;

        //printf("3\n");

        /*Actually read data*/
        ssize_t num_bytes = recvfrom(socket_desc,
                                     buffer, sizeof(buffer), 0,
                                     (struct sockaddr *)&client_addr,
                                     &client_len);

        //printf("Recvieved message\n");

        /*Only negative when error*/
        if (num_bytes < 0)
        {
            std::cerr << "recvfrom error: " << strerror(errno) << std::endl;
        }
        /*This is not used, but is supposed to ensure that the data sent is correct*/
        uint32_t crc = crc32bit(buffer + HEADER_SIZE, num_bytes - HEADER_SIZE);
        if (crc != ((DataHeader *)buffer)->crc || ((DataHeader *)buffer)->fragmentSize != num_bytes - HEADER_SIZE)
        {
            // Handle this later
        }
	    //printf("after crc\n");
        
        /*indicates the start of the data that is not the header*/
        char *payloadStart = buffer + HEADER_SIZE;

	    //printf("1\n");

        /* Insert data into the end*/
        received_data.insert(received_data.end(), payloadStart, payloadStart + ((DataHeader *)buffer)->fragmentSize);
        
        //printf("insert data\n");

        /*Append null to the end of the vector*/
        received_data.push_back('\0');

        /*Wipe data*/
        memset(info->client_message, '\0', 100000);

        /*Copies data*/
        memcpy(info->client_message, received_data.data(), received_data.size());

        // set robot_action_state to the first byte of the buffer header
        info->robot_action = *buffer;
        //printf("after seting data");
        info->flag = true;
        

        // std::cout << buffer << std::endl;
    }
    // Close the socket:
    close(socket_desc);

    return 0;
}
