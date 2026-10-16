#include "robot_serial_hardware/robot_serial_hardware.hpp"

#include <cmath>
#include <cstring>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "robot_serial_hardware/robot_command.hpp"

namespace robot_serial_hardware
{

hardware_interface::CallbackReturn RobotSerialHardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        return hardware_interface::CallbackReturn::ERROR;
    
    current_bus_pub_ = get_node()->create_publisher<serial_msgs::msg::CurrentBusVoltage>("current_bus_voltage", 10);
    temperature_pub_ = get_node()->create_publisher<serial_msgs::msg::Temperature>("temperature", 10);
    position_pub_ = get_node()->create_publisher<serial_msgs::msg::Position>("position", 10);

    auto port = info.hardware_info.hardware_parameters.at("port");
    auto baud = std::stoi(info.hardware_info.hardware_parameters.at("baudrate"));

    RCLCPP_INFO(
        rclcpp::get_logger("RobotSerialHardware"),
        "Opening serial port %s @ %d",
        port.c_str(),
        baud
    );

    if (serial_.open(port, baud))
    {
        RCLCPP_INFO(
            rclcpp::get_logger("RobotSerialHardware"),
            "Serial port opened successfully"
        );
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    else
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("RobotSerialHardware"),
            "Failed to open serial port"
        );
        return hardware_interface::CallbackReturn::ERROR;
    }
}


hardware_interface::CallbackReturn RobotSerialHardware::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(
        rclcpp::get_logger("RobotSerialHardware"),
        "Activating hardware"
    );

    for (const auto & [name, descr] : joint_command_interfaces_)
    {
        set_command(name, 0.0);
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}



hardware_interface::CallbackReturn RobotSerialHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(
        rclcpp::get_logger("RobotSerialHardware"),
        "Stopping hardware"
    );

    Command stop;

    stop.left = 0;
    stop.right = 0;

    serial_.write(stop);

    serial_.close();

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobotSerialHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    Command cmd;
    cmd.left = (get_command("front_left_wheel_joint/velocity")
    + get_command("rear_left_wheel_joint/velocity")) / 2.0;
    
    cmd.right = (get_command("front_right_wheel_joint/velocity")
    + get_command("rear_right_wheel_joint/velocity")) / 2.0;

    serial_.write(cmd);

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotSerialHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{

    Feedback feedback;

    serial_.get_feedback(feedback);

    serial_msgs::msg::CurrentBusVoltage current_bus_msg;
    current_bus_msg.front_left_wheel_current = feedback.current_bus_voltage.front_left_wheel_current;
    current_bus_msg.front_right_wheel_current = feedback.current_bus_voltage.front_right_wheel_current;
    current_bus_msg.back_left_wheel_current = feedback.current_bus_voltage.back_left_wheel_current;
    current_bus_msg.back_right_wheel_current = feedback.current_bus_voltage.back_right_wheel_current;
    current_bus_msg.front_drum_current = feedback.current_bus_voltage.front_drum_current;
    current_bus_msg.back_drum_current = feedback.current_bus_voltage.back_drum_current;
    current_bus_msg.front_actuator_current = feedback.current_bus_voltage.front_actuator_current;
    current_bus_msg.back_actuator_current = feedback.current_bus_voltage.back_actuator_current;
    current_bus_msg.main_battery_voltage = feedback.current_bus_voltage.main_battery_voltage;
    current_bus_msg.aux_battery_voltage = feedback.current_bus_voltage.aux_battery_voltage;

    serial_msgs::msg::Temperature temperature_msg;
    temperature_msg.front_left_wheel_temperature = feedback.temperature.front_left_wheel_temperature;
    temperature_msg.front_right_wheel_temperature = feedback.temperature.front_right_wheel_temperature;
    temperature_msg.back_left_wheel_temperature = feedback.temperature.back_left_wheel_temperature;
    temperature_msg.back_right_wheel_temperature = feedback.temperature.back_right_wheel_temperature;

    serial_msgs::msg::Position position_msg;
    position_msg.front_actuator_position = feedback.position.front_actuator_position;
    position_msg.back_actuator_position = feedback.position.back_actuator_position;

    current_bus_pub_->publish(current_bus_msg);
    temperature_pub_->publish(temperature_msg);
    position_pub_->publish(position_msg);

    return hardware_interface::return_type::OK;
}

}

PLUGINLIB_EXPORT_CLASS(
    robot_serial_hardware::RobotSerialHardware,
    hardware_interface::SystemInterface
)
