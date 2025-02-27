#include "client.hpp"
#include "frame_sender.hpp"

ConnectionHeaders create_connection_headers(const char *control_station_ip, int port)
{
    /* Create new client socket to send frame: */
    int client_socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (client_socket_fd < 0)
    {
        throw std::runtime_error("Error while creating socket");
    }

    /* Set port and IP for control station laptop: */
    struct sockaddr_in control_station_addr;
    socklen_t control_station_struct_len = sizeof(control_station_addr);
    control_station_addr.sin_family = AF_INET;
    control_station_addr.sin_port = htons(port);
    control_station_addr.sin_addr.s_addr = inet_addr(control_station_ip);

    ConnectionHeaders connection_headers = {client_socket_fd, control_station_addr};
    return connection_headers;
}

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

void client_send(const char *control_station_ip, unsigned char *data, size_t data_size)
{
    ConnectionHeaders connection_headers = create_connection_headers(control_station_ip, PORT);

    size_t sent_bytes = 0;
    uint16_t seqNo = 0;
    uint16_t totalChunks = data_size / CHUNK_SIZE;
    char sendBuffer[CHUNK_SIZE + HEADER_SIZE];
    memset(sendBuffer, '\0', sizeof(sendBuffer));

    while (sent_bytes < data_size)
    {
        size_t bytes_to_send = std::min(CHUNK_SIZE, (int)(data_size - sent_bytes));
        DataHeader header_struct;
        DataHeader *header = &header_struct;
        header->packetNum = seqNo;
        header->totalPackets = totalChunks;
        header->fragment_size = (uint16_t)bytes_to_send;
        header->crc = crc32bit((char *)(data + sent_bytes), bytes_to_send);
        memcpy(sendBuffer, header, HEADER_SIZE);
        memcpy(sendBuffer + HEADER_SIZE, data + sent_bytes, bytes_to_send);
        ssize_t transmission_result = sendto(connection_headers.client_socket_fd,
                                             sendBuffer,
                                             bytes_to_send + HEADER_SIZE, 0,
                                             (struct sockaddr *)&(connection_headers.control_station_addr),
                                             sizeof(connection_headers.control_station_addr));
        if (transmission_result < 0)
        {
            throw std::runtime_error("Unable to send message");
            return;
        }
        sent_bytes += bytes_to_send;
        seqNo++;
    }
    close(connection_headers.client_socket_fd);
}

void client_send(const char *control_station_ip, cv::Mat &image)
{
    ConnectionHeaders connection_headers = create_connection_headers(control_station_ip, PORT);
    send_frame(connection_headers, image);
    close(connection_headers.client_socket_fd);
}
