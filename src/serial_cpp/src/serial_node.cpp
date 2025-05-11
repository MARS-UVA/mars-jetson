#include "serial_node.hpp"
using namespace std::chrono_literals;

SerialNode::SerialNode(const rclcpp::NodeOptions& options) : 
Node("serial_cpp", options), 
serial_handler("/dev/ttyUSB0", 115200) {
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    teleop_subscriber_ = this->create_subscription<teleop_msgs::msg::MotorChanges>(
        "teleop", qos_profile, 
        std::bind(&SerialNode::update_currents, this, std::placeholders::_1));
    
    feedback_publisher_ = this->create_publisher<nucleo_msgs::msg::Feedback>(
        "feedback", qos_profile);
    
    send_timer_ = this->create_wall_timer(500ms, std::bind(&SerialNode::send_currents, this));
    read_timer_ = this->create_wall_timer(300ms, std::bind(&SerialNode::read_feedback, this));
    
}

void SerialNode::send_currents() {
    int msg_type = 1;
    boost::asio::write(serial_handler, boost::asio::buffer(&msg_type, 4));
    boost::asio::write(serial_handler, boost::asio::buffer(motorData));
}

void SerialNode::update_currents(teleop_msgs::msg::MotorChanges msg) {

}

void SerialNode::read_feedback() {

}

