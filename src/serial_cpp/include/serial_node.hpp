#pragma once

#include "nucleo_msgs/msg/feedback.hpp"
#include "teleop_msgs/msg/motor_changes.hpp"
#include "serial_handler.hpp"

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <chrono>
#include <memory>


class SerialNode : public rclcpp::Node {
    public:
        SerialNode(const rclcpp::NodeOptions& options);
        
    private:
        void send_currents();
        void update_currents(teleop_msgs::msg::MotorChanges msg);
        void read_feedback();
        int motorData[6] = {127, 127, 127, 127, 127, 127};
        SerialHandler serial_handler;

        // Subscribers and Publishers
        rclcpp::Subscription<teleop_msgs::msg::MotorChanges>::SharedPtr teleop_subscriber_;
        rclcpp::Publisher<nucleo_msgs::msg::Feedback>::SharedPtr feedback_publisher_;

        // Timers
        rclcpp::TimerBase::SharedPtr send_timer_;
        rclcpp::TimerBase::SharedPtr read_timer_;

};
