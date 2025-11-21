struct ThreadInfo{
    char client_message[1024];
    bool flag;
};

int create_server (ThreadInfo* info);

void client_send(unsigned char *data, size_t bytes_to_send);

struct DataHeader
{
    uint16_t dataType;
    uint16_t packetNum;
    uint16_t totalPackets;
    uint16_t fragment_size;
    uint16_t crc;
} __attribute__((packed));
#define HEADER_SIZE sizeof(DataHeader)
typedef struct DataHeader DataHeader;
