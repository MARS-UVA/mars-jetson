#pragma once
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <teleop_msgs/msg/motor_changes.hpp>

using namespace std::chrono_literals;

enum class RobotState {
    // Teleop state and autonomous states that return to teleop after completion. Also ESTOP
    TELEOP = 0,
    ESTOP = 1,
    DIG_ISOLATED = 2,
    DUMP_ISOLATED = 3,
    BREADCRUMBING_RECORDING = 4,
    BREADCRUMBING_EXECUTION = 5
    // Other states should be added here for the autonomous routines
    // TODO
};

class state_machine : public rclcpp::Node
{
public:
    explicit state_machine(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
private:
    rclcpp::TimerBase::SharedPtr timer_;
    // robot state is subscribed to update the isolated robotstates
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr robot_state_toggle_subscriber_;
    // receives motor commands and knows to send them based on the current state
    rclcpp::Subscription<teleop_msgs::msg::MotorChanges>::SharedPtr motor_command_subscriber_;

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr robot_state_publisher_;

    void robot_state_toggle_callback(const std_msgs::msg::UInt8::SharedPtr msg);
    void motor_command_callback(const teleop_msgs::msg::MotorChanges::SharedPtr msg);
    void timer_callback();

    RobotState robot_state;
};