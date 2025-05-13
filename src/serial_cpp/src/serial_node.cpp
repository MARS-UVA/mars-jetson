#include "serial_node.hpp"
using namespace std::chrono_literals;

SerialNode::SerialNode(): 
Node("serial_cpp"), 
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
    serial_handler.send(msg_type, motorData);
}

void SerialNode::update_currents(teleop_msgs::msg::MotorChanges::ConstSharedPtr msg) {
    std::vector<std::string> idx2motor = {"FL", "BL", "FR", "BR", "BucketSpeed", "BucketActuator"};
    std::vector<teleop_msgs::msg::AddMotor> adds = msg->adds;
    std::vector<teleop_msgs::msg::SetMotor> sets = msg->changes;
    for(auto s : sets) {
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Setting " << idx2motor[s.index] << "to" << s.velocity << std::endl);
        motorData[s.index] = s.velocity;
    }
    for(auto a : adds) {
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Adding " << a.vel_increment << "to" << idx2motor[a.index] << std::endl);
        motorData[a.index] += a.vel_increment;
        motorData[a.index] = std::max(motorData[a.index],0);
        motorData[a.index] = std::min(motorData[a.index],254);
    }
}

void SerialNode::read_feedback() {
    std::vector<float> feedback = serial_handler.readMsg();
    if(feedback.empty()) return;
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Sending Data: ");
    for(auto i : feedback) RCLCPP_DEBUG_STREAM(this->get_logger(), i<<" ");
    RCLCPP_DEBUG_STREAM(this->get_logger(), std::endl);
    nucleo_msgs::msg::Feedback msg;
    msg.front_left = feedback[0];
    msg.front_right = feedback[1];
    msg.back_left = feedback[2];
    msg.back_right = feedback[3];
    msg.l_drum = feedback[4];
    msg.r_drum = feedback[5];
    msg.l_actuator = feedback[6];
    msg.r_actuator = feedback[7];
    msg.actuator_height = feedback[8];
    feedback_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialNode>());
  rclcpp::shutdown();
}

