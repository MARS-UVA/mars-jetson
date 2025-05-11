#include <cstring>
#include <iostream>
#include <string>
#include <vector>
#include <sys/ioctl.h>
#include <boost/asio.hpp>

class SerialHandler {
    public:
        SerialHandler(std::string port, int baud) : io(), SER(io) {
            std::cout<<"make it past ser constructor"<<std::endl;
            try {
                SER.open(port);
                SER.set_option(boost::asio::serial_port_base::baud_rate(baud));
            }
            catch (const boost::system::system_error& e){
                throw std::runtime_error("SerialHandler: Unable to open serial port");
            }
            
        }

        std::vector<float> readMsg() {
            int bytesavailable;
            
            ioctl(SER.native_handle(), FIONREAD, &bytesavailable);
            if(bytesavailable < 40) return std::vector<float>();
            while(bytesavailable > 80) {
                std::vector<float> old_data(10);
                boost::asio::read(SER, boost::asio::buffer(&old_data, 40));
                ioctl(SER.native_handle(), FIONREAD, &bytesavailable);
            }

            int header;
            boost::asio::read(SER, boost::asio::buffer(&header, 4));
            std::vector<float> feedback(9); //fl, fr, bl, br, ldrum, rdrum, la, ra, actuator height
            boost::asio::read(SER, boost::asio::buffer(&feedback, 36));
            return feedback;
        }

        void send(int header, int data[6]) { //fl, bl, fr, br, drum, actuator
            boost::asio::write(SER, boost::asio::buffer(&header, 4));
            boost::asio::write(SER, boost::asio::buffer(&data, 24));
        }
    private:
        boost::asio::io_service io;
        boost::asio::serial_port SER;

};