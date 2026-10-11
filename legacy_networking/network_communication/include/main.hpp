#pragma once

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netinet/in.h>

#define HEADER_SIZE 10
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
