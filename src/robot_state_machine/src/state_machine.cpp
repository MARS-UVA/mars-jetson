#include "robot_state_machine/state_machine.hpp"


state_machine::state_machine(const rclcpp::NodeOptions & options) : rclcpp::Node("state_machine", options) {
    RCLCPP_INFO(this->get_logger(), "State Machine Node has started.");

    this->robot_state = RobotState::ESTOP; // Default is ESTOP
    timer_ = this->create_wall_timer(10ms, std::bind(&state_machine::timer_callback, this));
    robot_state_toggle_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
        "robot_state/toggle", 10, std::bind(&state_machine::robot_state_toggle_callback, this, std::placeholders::_1)
    );
    motor_command_subscriber_ = this->create_subscription<teleop_msgs::msg::MotorChanges>(
        "motor_commands", 10, std::bind(&state_machine::motor_command_callback, this, std::placeholders::_1)
    );
    robot_state_publisher_ = this->create_publisher<std_msgs::msg::UInt8>("robot_state", 10);
    motor_command_verified_publisher_ = this->create_publisher<teleop_msgs::msg::MotorChanges>("motor_commands_verified", 10);
}

void state_machine::timer_callback() {
    // Timer callback implementation
    std_msgs::msg::UInt8 msg;
    msg.data = static_cast<uint8_t>(this->robot_state);

    this->robot_state_publisher_->publish(msg);
}

void state_machine::robot_state_toggle_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received robot state toggle message: %d", msg->data);
    // Update robot state based on the received message and current state info
    this->robot_state = static_cast<RobotState>(msg->data);
}

void state_machine::motor_command_callback(const teleop_msgs::msg::MotorChanges::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received motor command message.");
    // Process motor command based on current state


    if (this->robot_state == RobotState::TELEOP) {
        // publish to motors
        this->motor_command_verified_publisher_->publish(*msg);
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<state_machine>());
  rclcpp::shutdown();
  return 0;
}
