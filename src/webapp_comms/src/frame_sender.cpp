#include "frame_sender.hpp"

void send_frame(ConnectionHeaders connectionHeaders, cv::Mat &image)
{
    int total_chunks, start, end;

    /* Frame compression and encoding */
    std::vector<unsigned char> buffer;
    std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 90};

    if (!cv::imencode(".jpg", image, buffer, compression_params))
    {
        std::cerr << "Image encoding failed!" << std::endl;
        return;
    }
    int buffer_size = buffer.size();
    total_chunks = (int)(buffer_size / CHUNK_SIZE) + (1 ? buffer_size % CHUNK_SIZE > 0 : 0);

    // Send data in chunks
    char *sendBuffer = (char *)malloc(CHUNK_SIZE + HEADER_SIZE);
    memset(sendBuffer, '\0', CHUNK_SIZE + HEADER_SIZE);
    for (int i = 0; i < total_chunks; i++)
    {
        start = i * CHUNK_SIZE;
        end = (start + CHUNK_SIZE > buffer_size) ? buffer_size : start + CHUNK_SIZE;

        DataHeader header = {i, (uint16_t)(end - start), crc32bit((char *)(buffer.data() + start), end - start)};
        memcpy(sendBuffer, &header, HEADER_SIZE);
        memcpy(sendBuffer + HEADER_SIZE, buffer.data() + start, end - start);

        ssize_t transmission_result = sendto(connectionHeaders.client_socket_fd,
                                             sendBuffer,
                                             (end - start) + HEADER_SIZE, 0,
                                             (struct sockaddr *)&(connectionHeaders.control_station_addr),
                                             sizeof(connectionHeaders.control_station_addr));
        if (transmission_result < 0)
        {
            throw std::runtime_error("Unable to send message");
            return;
        }
        memset(sendBuffer, '\0', CHUNK_SIZE + HEADER_SIZE);
    }
}
