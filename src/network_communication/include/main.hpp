#pragma once
#include <string.h>
#include <iostream>
#include <unistd.h>
#include <cstring>
#include <stdio.h>
#include <vector>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#define IMAGE_PORT 2000
#define CURRENT_FEEDBACK_PORT 2001
#define CHUNK_SIZE 1400

struct __attribute__((packed)) udpHeader {
    uint8_t reserved;
    uint8_t packetType;
    uint16_t packetLength;
    uint16_t numPackets;
    uint16_t batchPacketCount;
    uint16_t crc;
};
enum class HeaderFields {
    reserved = 0,
    packetType = 1,
    packetLength = 2,
    numPackets = 4,
    batchPacketCount = 6,
    crc = 8
};
