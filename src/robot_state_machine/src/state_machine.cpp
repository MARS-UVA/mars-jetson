#include "robot_state_machine/state_machine.hpp"


state_machine::state_machine(const rclcpp::NodeOptions & options) : rclcpp::Node("state_machine", options) {
    RCLCPP_INFO(this->get_logger(), "State Machine Node has started.");

    this->robot_state = RobotState::ESTOP; // Default is ESTOP
    this->breadcrumbing_data = {}; // Initialize breadcrumbing data

    timer_ = this->create_wall_timer(10ms, std::bind(&state_machine::timer_callback, this));

    robot_state_toggle_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
        "robot_state/toggle", 10, std::bind(&state_machine::robot_state_toggle_callback, this, std::placeholders::_1)
    );
    motor_command_subscriber_ = this->create_subscription<teleop_msgs::msg::MotorChanges>(
        "motor_commands", 10, std::bind(&state_machine::motor_command_callback, this, std::placeholders::_1)
    );

    robot_state_publisher_ = this->create_publisher<std_msgs::msg::UInt8>("robot_state", 10);
    motor_command_verified_publisher_ = this->create_publisher<teleop_msgs::msg::MotorChanges>("motor_commands_verified", 10);

    reset_breadcrumbing_service_ = this->create_service<robot_state_machine::srv::ResetBreadcrumbing>(
        "reset_breadcrumbing", std::bind(&state_machine::reset_breadcrumbing_callback, this, std::placeholders::_1, std::placeholders::_2)
    );
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
    if (this->robot_state == RobotState::ESTOP) {
        if (static_cast<RobotState>(msg->data) != RobotState::ESTOP) {
            RCLCPP_INFO(this->get_logger(), "Do not respond to state change request while in ESTOP. Ignoring message.");
            return;
        }
    } else if (static_cast<RobotState>(msg->data) == RobotState::ESTOP) {
        // TODO: Insert publisher to call activate button (trying to not have GPIO dependency in this node)
    } else if (static_cast<RobotState>(msg->data) == RobotState::BREADCRUMBING_EXECUTION) {
        // TODO: Call action_server to run breadcrumbing execution routine with passed in breadcrumbing data
    }
    this->robot_state = static_cast<RobotState>(msg->data);
}

void state_machine::motor_command_callback(const teleop_msgs::msg::MotorChanges::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received motor command message.");
    // Process motor command based on current state


    if (this->robot_state == RobotState::TELEOP) {
        // publish to motors
        this->motor_command_verified_publisher_->publish(*msg);
    } else if (this->robot_state == RobotState::BREADCRUMBING_RECORDING) {
        // store motor command for breadcrumbing and publish to motors
        this->motor_command_verified_publisher_->publish(*msg);
        auto data_point = std::make_shared<BreadcrumbingDataPoint>();
        data_point->motor_command = *msg;
        data_point->timestamp = this->now();
        this->breadcrumbing_data.push_back(data_point);
    } else {
        RCLCPP_WARN(this->get_logger(), "Received motor command in non-teleop, non-breadcrumbing state. Command ignored.");
    }
}

void state_machine::reset_breadcrumbing_callback(const std::shared_ptr<robot_state_machine::srv::ResetBreadcrumbing::Request> request,
    std::shared_ptr<robot_state_machine::srv::ResetBreadcrumbing::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Received reset breadcrumbing service call.");
    // Reset breadcrumbing data and state
    if (request->reset_breadcrumbing) {
        RCLCPP_INFO(this->get_logger(), "Resetting breadcrumbing data.");
        this->breadcrumbing_data.clear();
    } else {
        RCLCPP_WARN(this->get_logger(), "Received reset breadcrumbing request with reset_breadcrumbing set to false. No action taken.");
    }
    response->success = "Breadcrumbing array size: " + std::to_string(this->breadcrumbing_data.size());

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<state_machine>());
  rclcpp::shutdown();
  return 0;
}
