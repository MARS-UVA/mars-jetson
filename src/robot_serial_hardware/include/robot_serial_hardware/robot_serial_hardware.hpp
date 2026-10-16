#pragma once

#include "hardware_interface/system_interface.hpp"
#include "robot_serial_hardware/serial.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "serial_msgs/msg/current_bus_voltage.hpp"
#include "serial_msgs/msg/temperature.hpp"
#include "serial_msgs/msg/position.hpp"

namespace robot_serial_hardware
{

class RobotSerialHardware : public hardware_interface::SystemInterface
{
    public:

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & info) override;

        hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

        hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

        rclcpp::Publisher<serial_msgs::msg::CurrentBusVoltage>::SharedPtr current_bus_pub_;
        rclcpp::Publisher<serial_msgs::msg::Temperature>::SharedPtr temperature_pub_;
        rclcpp::Publisher<serial_msgs::msg::Position>::SharedPtr position_pub_;
    private:

        double fl_command_;
        double fr_command_;
        double rl_command_;
        double rr_command_;

        double fl_position_;
        double fr_position_;
        double rl_position_;
        double rr_position_;

        double fl_velocity_;
        double fr_velocity_;
        double rl_velocity_;
        double rr_velocity_;

        Serial serial_;

};

}
