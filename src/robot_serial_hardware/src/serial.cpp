#include "robot_serial_hardware/serial.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>

namespace robot_serial_hardware
{

Serial::Serial() : serial_(io_)
{
}

Serial::~Serial()
{
    close();
}

bool Serial::open(const std::string &port, int baudrate)
{
    try {
        serial_.open(port);
        serial_.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
        serial_.set_option(boost::asio::serial_port_base::character_size(8));
        serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    } catch (const boost::system::system_error &e) {
        RCLCPP_ERROR(
            rclcpp::get_logger("Serial"),
            "Failed to open serial port: %s",
            e.what()
        );
        return false;
    }

    running_ = true;
    read_thread_ = std::thread(&Serial::read_loop, this);

    return true;
}

void Serial::close()
{
    running_ = false;

    if (serial_.is_open())
        serial_.close();

    if (read_thread_.joinable())
        read_thread_.join();

}

bool Serial::write(Command & cmd)
{
    if (cmd.left < -1.0) cmd.left = -1.0;
    if (cmd.left > 1.0) cmd.left = 1.0;
    if (cmd.right < -1.0) cmd.right = -1.0;
    if (cmd.right > 1.0) cmd.right = 1.0;

    uint8_t left = static_cast<uint8_t>((cmd.left + 1.0) * 127);
    uint8_t right = static_cast<uint8_t>((cmd.right + 1.0) * 127);

    uint8_t data[3] = {0xFF, left, right};

    try {
        boost::asio::write(serial_, boost::asio::buffer(data, sizeof(data)));
    } catch (const boost::system::system_error &e) {
        if (!running_)
            return false;

        RCLCPP_ERROR(
            rclcpp::get_logger("Serial"),
            "Failed to write to serial port: %s",
            e.what()
        );
        return false;
    }

    return true;
}

void Serial::get_feedback(Feedback & feedback)
{
    std::lock_guard<std::mutex> lock(read_mutex_);
    feedback = feedback_;
}

void Serial::read_loop()
{
    uint8_t read_buffer[(size_t) (2 + fmax(fmax(sizeof(CurrentBusVoltageFeedback), sizeof(TemperatureFeedback)), sizeof(PositionFeedback)))];
    while (running_)
    {
        boost::system::error_code error;
        
        size_t bytes_read = boost::asio::read(serial_, boost::asio::buffer(&read_buffer[0], 1), error);
        if (error)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("Serial"),
                "Error reading from serial port: %s",
                error.message().c_str()
            );
            return;
        }

        if (bytes_read != 1 || read_buffer[0] != 0xFF)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("Serial"),
                "Invalid start byte read from serial port: %02X",
                read_buffer[0]
            );
            continue;
        }

        size_t bytes_read_header = boost::asio::read(serial_, boost::asio::buffer(&read_buffer[1], 1), error);
        if (error)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("Serial"),
                "Error reading from serial port: %s",
                error.message().c_str()
            );
            return;
        }

        if (bytes_read_header != 1)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("Serial"),
                "Failed to read feedback header from serial port"
            );
            continue;
        }

        // Read the rest of the feedback data, depending on the header type
        size_t feedback_size = 0;
        switch (read_buffer[1])
        {
            case CURRENT_FEEDBACK_HEADER:
                feedback_size = sizeof(CurrentBusVoltageFeedback);
                break;
            case TEMPERATURE_FEEDBACK_HEADER:
                feedback_size = sizeof(TemperatureFeedback);
                break;
            case POSITION_FEEDBACK_HEADER:
                feedback_size = sizeof(PositionFeedback);
                break;
            default:
                RCLCPP_ERROR(
                    rclcpp::get_logger("Serial"),
                    "Unknown feedback header: %d",
                    read_buffer[1]
                );
                continue;
        }

        size_t bytes_read_data = boost::asio::read(serial_, boost::asio::buffer(&read_buffer[2], feedback_size), error);
        
        handle_read(read_buffer);
    }
}

void Serial::handle_read(uint8_t* read_buffer)
{
    Feedback feedback;
    feedback.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
    feedback.header = read_buffer[1];

    switch (feedback.header)
    {
        case CURRENT_FEEDBACK_HEADER:
            std::memcpy(&feedback.current_bus_voltage, &read_buffer[2], sizeof(CurrentBusVoltageFeedback));
            break;
        case TEMPERATURE_FEEDBACK_HEADER:
            std::memcpy(&feedback.temperature, &read_buffer[2], sizeof(TemperatureFeedback));
            break;
        case POSITION_FEEDBACK_HEADER:
            std::memcpy(&feedback.position, &read_buffer[2], sizeof(PositionFeedback));
            break;
        default:
            RCLCPP_ERROR(
                rclcpp::get_logger("Serial"),
                "Unknown feedback header: %d",
                feedback.header
            );
            return;
    }

    {
        std::lock_guard<std::mutex> lock(read_mutex_);
        feedback_ = feedback;
    }
}

}
