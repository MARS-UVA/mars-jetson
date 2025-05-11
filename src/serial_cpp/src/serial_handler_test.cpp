#include "../include/serial_handler.hpp"
#include <thread>
#include <chrono>

int main() {
    int header = 0;
    int data[6] = {127, 127, 127, 127, 127, 127};
    std::cout<<"ran"<<std::endl;
    SerialHandler serial_handler("/dev/ttyUSB0", 115200);
    
    while(true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        serial_handler.send(header, data);
        std::cout<<"sent data"<< std::endl;
        auto recv_data = serial_handler.readMsg();
        std::cout<<"reading data: ";
        for(int i=0; i<recv_data.size(); i++) {
            std::cout<<recv_data[i];
        }
        std::cout<<std::endl;
    }

}