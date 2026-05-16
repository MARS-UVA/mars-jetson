#pragma once

#include <iostream>
#include <chrono>
#include <vector>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <teleop_msgs/msg/motor_changes.hpp>
#include "robot_state_machine/srv/reset_breadcrumbing.hpp"

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

typedef struct {
    // Add fields as necessary to track breadcrumbing data
    // For example, you might want to store a vector of motor commands with timestamps
    teleop_msgs::msg::MotorChanges motor_command;
    rclcpp::Time timestamp;
} BreadcrumbingDataPoint;

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
    rclcpp::Publisher<teleop_msgs::msg::MotorChanges>::SharedPtr motor_command_verified_publisher_;

    rclcpp::Service<robot_state_machine::srv::ResetBreadcrumbing>::SharedPtr reset_breadcrumbing_service_;

    void robot_state_toggle_callback(const std_msgs::msg::UInt8::SharedPtr msg);
    void motor_command_callback(const teleop_msgs::msg::MotorChanges::SharedPtr msg);
    void timer_callback();
    void reset_breadcrumbing_callback(const std::shared_ptr<robot_state_machine::srv::ResetBreadcrumbing::Request> request,
        std::shared_ptr<robot_state_machine::srv::ResetBreadcrumbing::Response> response);

    RobotState robot_state;
    std::vector<std::shared_ptr<BreadcrumbingDataPoint>> breadcrumbing_data; // Array to store motor commands for breadcrumbing 
};