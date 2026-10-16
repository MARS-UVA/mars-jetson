#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include "robot_serial_hardware/robot_command.hpp"
#include "boost/asio.hpp"

namespace robot_serial_hardware
{

class Serial
{
public:

    Serial();
    ~Serial();

    bool open(const std::string & port, int baudrate);
    void close();
    bool write(Command & cmd);
    void get_feedback(Feedback & feedback);

private:

    std::thread read_thread_;
    std::atomic<bool> running_{false};
    boost::asio::io_context io_;
    boost::asio::serial_port serial_{io_};
    std::mutex read_mutex_;
    Feedback feedback_;
    void read_loop();
    void handle_read(uint8_t* read_buffer);
};

}
