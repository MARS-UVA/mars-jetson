#include <iostream>

#include "teleop_msgs/msg/gamepad_state.hpp"

int main()
{
  teleop_msgs::msg::GamepadState state;

  std::cout << state.a_pressed << std::endl;

  return 0;
}
