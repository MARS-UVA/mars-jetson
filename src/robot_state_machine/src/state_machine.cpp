#include "robot_state_machine/state_machine.hpp"

class state_machine : public rclcpp::Node
{
    public:
        state_machine() : Node("state_machine") {
            RCLCPP_INFO(this->get_logger(), "State Machine Node has started.");
        }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<state_machine>());
  rclcpp::shutdown();
  return 0;
}
