#include <catch2/catch_all.hpp>
#include "../src/client.hpp"
#include "../src/server.hpp"
#include <vector>
#include <thread>
#include <future>
#include <chrono>
#include <iostream>
#include <cstring>
#include <fcntl.h>
#include <chrono>

#include "../../platform_compat.h"

#define FEEDBACK_PORT 2001
#define VIDEO_PORT 2000
#define WEBUI_PORT 8080

int main(int argc, char *argv[]){
    std::cout << argc << std::endl;
    std::cout << *argv[1] << std::endl;
    std::cout << *argv[2] << std::endl;
    if(argc == 3 && *argv[1] && *argv[2]){
        if(*argv[1] == '1'){
            unsigned char msg[] = "Hellow eorld";
            std::cout << "Sending Message" << std::endl;
            client_send(argv[2], msg, sizeof(msg), FEEDBACK_PORT);
        }
        else if(*argv[1] == '2'){
            cv::Mat imageMatrix = cv::imread("../assets/test1.png", cv::IMREAD_COLOR);
            std::vector<unsigned char> imgBuffer;
            std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 90};
            cv::imencode(".jpg", imageMatrix, imgBuffer, compression_params);
            std::cout << "Sending Image" << std::endl;
            client_send(argv[2], imageMatrix, VIDEO_PORT);
        }
        else if(*argv[1] == '3'){
            ThreadInfo info;
            info.flag = false;
            memset(info.client_message, '\0', sizeof(info.client_message));
        
            std::thread socket(create_server, &info);
        
            // unsigned char data[] = "Hello, world!";
            // size_t data_size = sizeof(data) - 1;
            // const char *ip = "192.168.0.200";
        
            // usleep(1000);
        
            // client_send(ip, data, data_size, 8080);
        
            while(info.flag == false); {}
            std::cout << info.client_message << std::endl;
        
            socket.join();
        }
    }
}