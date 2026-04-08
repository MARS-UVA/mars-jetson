/* Sends communcations to the control station */
#include "client.hpp"

/* IP based, this is an environmental variable */
const char* CONTROL_STATION_IP_FOR_CLIENT = std::getenv("CONTROL_STATION_IP");

//const char* CONTROL_STATION_IP_FOR_CLIENT = "192.168.0.100";
//const char* CONTROL_STATION_IP = "192.168.0.200";
ConnectionHeaders create_connection_headers(int port)
{
    // Ensure that the environmental variable is set, otherwise abort and exit
    if (CONTROL_STATION_IP_FOR_CLIENT == nullptr) {
        std::cerr << "Error: CONTROL_STATION_IP environment variable not set" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    /* Create new client socket to send frame:
    AF_INET: Address Family, specifies how the address is actually formatted; In this case IPv4
    SOCK_DGRAM: Send as a datagram, so it will be connectionless
    IPPROTO_UDP: Use the UDP Protocol 
    */
    int client_socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    /* Ensure the socket was actually created */
    if (client_socket_fd < 0)
    {
        throw std::runtime_error("Error while creating socket");
    }

    /* Set port and IP for control station laptop: */
    struct sockaddr_in control_station_addr;
    socklen_t control_station_struct_len = sizeof(control_station_addr);

    /*Specifies address family, again IPv4 */
    control_station_addr.sin_family = AF_INET;

    /* Specifies what the port is stored in network byte order: see https://stackoverflow.com/questions/19207745/htons-function-in-socket-programing for more details */
    control_station_addr.sin_port = htons(port);

    //std::cout << "Control station ip: " << CONTROL_STATION_IP_FOR_CLIENT << std::endl;

    /* Add the actual IP address */
    control_station_addr.sin_addr.s_addr = inet_addr(CONTROL_STATION_IP_FOR_CLIENT);

    /* Specify the socket that it will be sending on and the destination */
    ConnectionHeaders connection_headers = {client_socket_fd, control_station_addr};

    /* Return the source socket and the destionation port */
    return connection_headers;
}

//According to Eric Z (~April 3), this is not used, but I believe it is a checksum to ensure that data was not lost
uint32_t crc32bit(const char *data, size_t data_size)
{
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < data_size; i++)
    {
        // std::cout << "Data: " << data[i] << std::endl;
        crc = crc ^ data[i];
        for (size_t j = 0; j < 8; j++)
        {
            if (crc & 1)
            {
                crc = (crc >> 1) ^ 0xEDB88320;
            }
            else
            {
                crc = crc >> 1;
            }
        }
    }
    return ~crc;
}

void client_send(unsigned char *data, size_t data_size, int server_port)
{
    /*use connection_headers */
    ConnectionHeaders connection_headers = create_connection_headers(server_port);

    /* Track how much has been sent so far in terms of data size, not the number of chunks */
    size_t sent_bytes = 0;
    
    /* Track the correct order of the packets sent*/
    uint16_t seqNo = 0;

    /* Calculates the total number of chunks that will actually be sent 
    Logic: integer division truncates, so add 1 if there is a remainder
    CHUNK_SIZE defined in main.hpp
    */
    uint16_t totalChunks = data_size / CHUNK_SIZE + (data_size % CHUNK_SIZE > 0 ? 1 : 0);
    /* Does this send the chunks and the headers separately? */
    char sendBuffer[CHUNK_SIZE + HEADER_SIZE];
    memset(sendBuffer, '\0', sizeof(sendBuffer));

    // std::cout << "Sending feedback bytes..." << std::endl;
    /*Keeps track of how size of data sent vs how how much needs to be sent*/
    while (sent_bytes < data_size)
    {
        /*Specify the size of the bytes to send*/
        size_t bytes_to_send = std::min(CHUNK_SIZE, (int)(data_size - sent_bytes));
        /*Create the headers*/
        DataHeader header_struct;
        DataHeader *header = &header_struct;
        header->packetToSend = seqNo;
        header->totalPacketsToSend = totalChunks;
        header->fragmentSize = (uint16_t)bytes_to_send;
        header->crc = crc32bit((char *)(data + sent_bytes), bytes_to_send);
        memcpy(sendBuffer, header, HEADER_SIZE);
        memcpy(sendBuffer + HEADER_SIZE, data + sent_bytes, bytes_to_send);
        
        /*Sends the packet through port specified prior
        https://pubs.opengroup.org/onlinepubs/009604499/functions/sendto.html
        Inputs:
            int socket: specifies socket [connection_headers.client_socket_fd]
            const void *message: points to a buffer containing message [sendBuffer]
            length: size in bytes [bytes_to_send + HEADER_SIZE]
            int flags: message transmission type [0]
            dest_addr: destination address, must be sockaddr struct [(struct sockaddr * ) ...]
            dest_len: specifies length of sockaddr structure in dest_addr
        */
        ssize_t transmission_result = sendto(connection_headers.client_socket_fd,
                                             sendBuffer,
                                             bytes_to_send + HEADER_SIZE, 0,
                                             (struct sockaddr *)&(connection_headers.control_station_addr),
                                             sizeof(connection_headers.control_station_addr));
        /* transmission_result actually records the number of bytes_sent, and returns -1 of there was an error */
        if (transmission_result < 0)
        {
            throw std::runtime_error("Unable to send message");
            return;
        }
        /*Increment Loop*/
        sent_bytes += bytes_to_send;
        /*Increment what part of the sequences this is*/
        seqNo++;
    }
    close(connection_headers.client_socket_fd);
}