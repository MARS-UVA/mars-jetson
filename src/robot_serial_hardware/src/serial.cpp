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

double clamp(double value, double min, double max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

uint8_t to_byte(double value)
{
    return static_cast<uint8_t>((value + 1.0) * 127);
}

bool Serial::write(Command & cmd)
{
    cmd.front_left = clamp(cmd.front_left, -1.0, 1.0);
    cmd.front_right = clamp(cmd.front_right, -1.0, 1.0);
    cmd.rear_left = clamp(cmd.rear_left, -1.0, 1.0);
    cmd.rear_right = clamp(cmd.rear_right, -1.0, 1.0);
    cmd.front_drum = clamp(cmd.front_drum, -1.0, 1.0);
    cmd.front_arm = clamp(cmd.front_arm, -1.0, 1.0);
    cmd.rear_drum = clamp(cmd.rear_drum, -1.0, 1.0);
    cmd.rear_arm = clamp(cmd.rear_arm, -1.0, 1.0);

    uint8_t front_left = to_byte(cmd.front_left);
    uint8_t front_right = to_byte(cmd.front_right);
    uint8_t rear_left = to_byte(cmd.rear_left);
    uint8_t rear_right = to_byte(cmd.rear_right);
    uint8_t front_drum = to_byte(cmd.front_drum);
    uint8_t front_arm = to_byte(cmd.front_arm);
    uint8_t rear_drum = to_byte(cmd.rear_drum);
    uint8_t rear_arm = to_byte(cmd.rear_arm);

    // Start byte (0xFF), followed by header (0x00), followed by the command data
    uint8_t data[10] = {0xFF, 0x00, front_left, rear_left, front_right, rear_right, front_drum, front_arm, rear_drum, rear_arm};
    RCLCPP_DEBUG(
        rclcpp::get_logger("Serial"),
        "Writing to serial port: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
        data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9]
    );

    try {
        std::lock_guard<std::mutex> lock(write_mutex_);
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

void Serial::get_feedback(Feedback & current_bus_voltage_feedback, Feedback & temperature_feedback, Feedback & position_feedback)
{
    std::lock_guard<std::mutex> lock(read_mutex_);
    current_bus_voltage_feedback = current_bus_voltage_feedback_;
    temperature_feedback = temperature_feedback_;
    position_feedback = position_feedback_;
}

void Serial::read_loop()
{
    std::vector<uint8_t> buffer;
    buffer.reserve(256);
    uint8_t temp_buf[64];

    while (running_)
    {
        boost::system::error_code error;
        // Read available bytes
        size_t bytes_transferred = serial_.read_some(boost::asio::buffer(temp_buf), error);
        
        if (error)
        {
            if (running_) {
                RCLCPP_ERROR(rclcpp::get_logger("Serial"), "Serial read error: %s", error.message().c_str());
            }
            break;
        }

        buffer.insert(buffer.end(), temp_buf, temp_buf + bytes_transferred);

        // Slide window until we find valid header [0xFF, header, 0x00, 0x00]
        while (buffer.size() >= 4)
        {
            if (buffer[0] != 0xFF || buffer[2] != 0x00 || buffer[3] != 0x00)
            {
                buffer.erase(buffer.begin()); // Resync by dropping one byte
                continue;
            }

            uint8_t header = buffer[1];
            size_t payload_size = 0;

            switch (header)
            {
                case CURRENT_FEEDBACK_HEADER:
                    payload_size = sizeof(CurrentBusVoltageFeedback);
                    break;
                case TEMPERATURE_FEEDBACK_HEADER:
                    payload_size = sizeof(TemperatureFeedback);
                    break;
                case POSITION_FEEDBACK_HEADER:
                    payload_size = sizeof(PositionFeedback);
                    break;
                default:
                    // Invalid header, discard start byte and resync
                    buffer.erase(buffer.begin());
                    continue;
            }

            size_t total_frame_size = 4 + payload_size;
            if (buffer.size() < total_frame_size)
            {
                // Wait for the full packet to arrive
                break;
            }

            // Extract and parse
            handle_read(buffer.data());

            // Consume packet from buffer
            buffer.erase(buffer.begin(), buffer.begin() + total_frame_size);
        }
    }
}

void Serial::handle_read(const uint8_t* read_buffer)
{
    std::lock_guard<std::mutex> lock(read_mutex_);

    Feedback *feedback_ = nullptr;

    switch (read_buffer[1])
    {
        case CURRENT_FEEDBACK_HEADER:
            feedback_ = &current_bus_voltage_feedback_;
            break;
        case TEMPERATURE_FEEDBACK_HEADER:
            feedback_ = &temperature_feedback_;
            break;
        case POSITION_FEEDBACK_HEADER:
            feedback_ = &position_feedback_;
            break;
        default:
            RCLCPP_WARN(
                rclcpp::get_logger("Serial"),
                "Received unknown header: %02X",
                read_buffer[1]
            );
            return;
    }

    feedback_->timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
    feedback_->header = read_buffer[1];

    switch (feedback_->header)
    {
        case CURRENT_FEEDBACK_HEADER:
            std::memcpy(&feedback_->current_bus_voltage, &read_buffer[4], sizeof(CurrentBusVoltageFeedback));
            break;
        case TEMPERATURE_FEEDBACK_HEADER:
            std::memcpy(&feedback_->temperature, &read_buffer[4], sizeof(TemperatureFeedback));
            break;
        case POSITION_FEEDBACK_HEADER:
            std::memcpy(&feedback_->position, &read_buffer[4], sizeof(PositionFeedback));
            break;
    }
}

}
