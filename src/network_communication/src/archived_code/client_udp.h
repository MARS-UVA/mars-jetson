#ifndef CLIENT_H
#define CLIENT_H

#ifdef __cplusplus
extern "C"
{
#endif

    void send_frame(const char *ip, unsigned char *data, size_t data_size);

#ifdef __cplusplus
}
#endif

#endif