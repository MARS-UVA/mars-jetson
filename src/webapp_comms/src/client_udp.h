#ifndef CLIENT_H
#define CLIENT_H

#ifdef __cplusplus
extern "C"
{
#endif

    void create_sending_client(char *ip, unsigned char *data, size_t data_size);

#ifdef __cplusplus
}
#endif

#endif