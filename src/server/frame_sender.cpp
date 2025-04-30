#include "frame_sender.hpp"

void send_frame(ConnectionHeaders connectionHeaders, cv::Mat &image)
{
    //std::cout << "inside frame sender" << std::endl;
    uint16_t total_chunks;
    uint32_t start, end;

    /* Frame compression and encoding */
    std::vector<unsigned char> buffer;
    std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 90};

    if (!cv::imencode(".jpg", image, buffer, compression_params))
    {
        std::cerr << "Image encoding failed!" << std::endl;
        return;
    }
    int buffer_size = buffer.size();
    //std::cout << "Buffer size: " << buffer_size << std::endl;
    total_chunks = (buffer_size / CHUNK_SIZE) + ((buffer_size % CHUNK_SIZE) > 0 ? 1 : 0);

    // Send data in chunks
    char *sendBuffer = (char *)malloc(CHUNK_SIZE + HEADER_SIZE);
    memset(sendBuffer, '\0', CHUNK_SIZE + HEADER_SIZE);
    for (uint16_t i = 0; i < total_chunks; i++)
    {
        //std::cout << i+1 << " out of " << total_chunks << std::endl;
        start = i * CHUNK_SIZE;
        end = (start + CHUNK_SIZE >= buffer_size) ? buffer_size : start + CHUNK_SIZE;

        DataHeader header = {i, total_chunks, (uint16_t)(end - start), crc32bit((char *)(buffer.data() + start), (uint16_t)(end - start))};
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
        usleep(100);
    }
    free(sendBuffer);
}
