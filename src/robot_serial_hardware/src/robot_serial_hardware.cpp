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

    stop.front_left = 0;
    stop.front_right = 0;
    stop.rear_left = 0;
    stop.rear_right = 0;
    stop.front_drum = 0;
    stop.front_arm = 0;
    stop.rear_drum = 0;
    stop.rear_arm = 0;

    serial_.write(stop);

    serial_.close();

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobotSerialHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    Command cmd;
    cmd.front_left = get_command("front_left_wheel_joint/velocity");
    cmd.front_right = get_command("front_right_wheel_joint/velocity");
    cmd.rear_left = get_command("rear_left_wheel_joint/velocity");
    cmd.rear_right = get_command("rear_right_wheel_joint/velocity");
    cmd.front_drum = get_command("front_drum_joint/velocity");
    cmd.front_arm = get_command("front_arm_joint/velocity");
    cmd.rear_drum = get_command("rear_drum_joint/velocity");
    cmd.rear_arm = get_command("rear_arm_joint/velocity");

    serial_.write(cmd);

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotSerialHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{

    Feedback current_bus_voltage_feedback;
    Feedback temperature_feedback;
    Feedback position_feedback;

    serial_.get_feedback(current_bus_voltage_feedback, temperature_feedback, position_feedback);

    serial_msgs::msg::CurrentBusVoltage current_bus_msg;
    current_bus_msg.front_left_wheel_current = current_bus_voltage_feedback.current_bus_voltage.front_left_wheel_current;
    current_bus_msg.front_right_wheel_current = current_bus_voltage_feedback.current_bus_voltage.front_right_wheel_current;
    current_bus_msg.back_left_wheel_current = current_bus_voltage_feedback.current_bus_voltage.rear_left_wheel_current;
    current_bus_msg.back_right_wheel_current = current_bus_voltage_feedback.current_bus_voltage.rear_right_wheel_current;
    current_bus_msg.front_drum_current = current_bus_voltage_feedback.current_bus_voltage.front_drum_current;
    current_bus_msg.back_drum_current = current_bus_voltage_feedback.current_bus_voltage.rear_drum_current;
    current_bus_msg.front_actuator_current = current_bus_voltage_feedback.current_bus_voltage.front_actuator_current;
    current_bus_msg.back_actuator_current = current_bus_voltage_feedback.current_bus_voltage.rear_actuator_current;
    current_bus_msg.main_battery_voltage = current_bus_voltage_feedback.current_bus_voltage.main_battery_voltage;
    current_bus_msg.aux_battery_voltage = current_bus_voltage_feedback.current_bus_voltage.aux_battery_voltage;
    current_bus_pub_->publish(current_bus_msg);

    serial_msgs::msg::Temperature temperature_msg;
    temperature_msg.front_left_wheel_temperature = temperature_feedback.temperature.front_left_wheel_temperature;
    temperature_msg.front_right_wheel_temperature = temperature_feedback.temperature.front_right_wheel_temperature;
    temperature_msg.back_left_wheel_temperature = temperature_feedback.temperature.rear_left_wheel_temperature;
    temperature_msg.back_right_wheel_temperature = temperature_feedback.temperature.rear_right_wheel_temperature;
    temperature_msg.front_drum_temperature = temperature_feedback.temperature.front_drum_temperature;
    temperature_msg.back_drum_temperature = temperature_feedback.temperature.rear_drum_temperature;
    temperature_pub_->publish(temperature_msg);

    serial_msgs::msg::Position position_msg;
    position_msg.front_actuator_position = position_feedback.position.front_actuator_position;
    position_msg.back_actuator_position = position_feedback.position.rear_actuator_position;
    position_pub_->publish(position_msg);

    return hardware_interface::return_type::OK;
}

}

PLUGINLIB_EXPORT_CLASS(
    robot_serial_hardware::RobotSerialHardware,
    hardware_interface::SystemInterface
)
