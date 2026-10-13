#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <thread>
#include <unistd.h>

#define MESSAGE_LENGTH 1500

void send_file(const std::string& server_ip, int server_port, const std::string& file_path) {
    int sock;
    struct sockaddr_in server_addr;

    // Create a UDP socket
    if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Socket creation failed");
        return;
    }

    // Set server address
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(server_port);
    inet_pton(AF_INET, server_ip.c_str(), &server_addr.sin_addr);

    // Open the file
    std::ifstream file(file_path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file " << file_path << std::endl;
        close(sock);
        return;
    }

    // Read file content into buffer
    std::streamsize file_size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<char> file_data(file_size);
    if (!file.read(file_data.data(), file_size)) {
        std::cerr << "Error: Unable to read file " << file_path << std::endl;
        close(sock);
        return;
    }

    // Split file into packets
    int total_packets = (file_size + MESSAGE_LENGTH - 4 - 1) / (MESSAGE_LENGTH - 3); // 3 bytes for headers
    std::cout << "Sending file in " << total_packets << " packets..." << std::endl;

    for (int i = 0; i < total_packets; ++i) {
        std::vector<char> packet(MESSAGE_LENGTH);

        // Add sequence number (1 byte) and total packets (2 bytes)
        packet[0] = static_cast<char>(i); // Sequence number
        packet[1] = static_cast<char>((total_packets >> 8) & 0xFF); // High byte of total packets
        packet[2] = static_cast<char>(total_packets & 0xFF);        // Low byte of total packets

        // Add chunk data
        int chunk_start = i * (MESSAGE_LENGTH - 3);
        int chunk_size = std::min(static_cast<int>(file_size - chunk_start), MESSAGE_LENGTH - 3);
        memcpy(packet.data() + 3, file_data.data() + chunk_start, chunk_size);

        // Send packet
        if (sendto(sock, packet.data(), chunk_size + 3, 0, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            perror("Failed to send packet");
            close(sock);
            return;
        }

        std::cout << "Sent packet " << i + 1 << " of " << total_packets << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    }

    std::cout << "File sent successfully!" << std::endl;

    close(sock);
}

int main() {
    std::string server_ip = "127.0.0.1";
    int server_port = 8080;
    std::string file_path = "test_image.jpg"; // Replace with the path to the file you want to send

    send_file(server_ip, server_port, file_path);

    return 0;
}
