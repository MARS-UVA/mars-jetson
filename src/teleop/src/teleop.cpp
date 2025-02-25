#include <functional>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "teleop/control.hpp"
#include "teleop_msgs/msg/gamepad_state.hpp"

namespace teleop {

class TeleopNode : public rclcpp::Node {

    rclcpp::Subscription<teleop_msgs::msg::GamepadState>::SharedPtr gamepad_state_subscription;

    std::unique_ptr<DriveControlStrategy> drive_control_strategy;

public:

    TeleopNode(std::unique_ptr<DriveControlStrategy> drive_control_strategy)
    : Node("teleop"), drive_control_strategy(std::move(drive_control_strategy)) {
        using std::placeholders::_1;
        gamepad_state_subscription = create_subscription<teleop_msgs::msg::GamepadState>(
            "gamepad_state",
            10,
            std::bind(&on_receive_gamepad_state, this, _1)
        );
    }

    void set_drive_control_strategy(std::unique_ptr<DriveControlStrategy> drive_control_strategy) {
        this->drive_control_strategy = std::move(drive_control_strategy);
    }

private:

    void on_receive_gamepad_state(const teleop_msgs::msg::GamepadState& gamepad_state) {
        teleop::WheelSpeeds wheel_speeds = drive_control_strategy->get_wheel_speeds(gamepad_state);

        RCLCPP_INFO(this->get_logger(), "Calculated: %s", static_cast<std::string>(wheel_speeds).c_str());

        // TODO: Finish implementing TeleopNode::on_receive_gamepad_state
    }

};

}  /* namespace teleop */

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<teleop::TeleopNode>());
    rclcpp::shutdown();
    return 0;
}
