#ifndef FRAME_SENDER_H
#define FRAME_SENDER_H

void usleep_simulation(unsigned int microseconds);

#define PORT 2000
#define CHUNK_SIZE 1450
#define MAX_PAYLOAD_SIZE CHUNK_SIZE - HEADER_SIZE

void send_frame(const char *control_station_ip, cv::Mat &image);

#endif
