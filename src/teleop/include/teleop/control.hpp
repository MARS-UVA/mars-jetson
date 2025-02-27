#ifndef TELEOP_CONTROL_HPP
#define TELEOP_CONTROL_HPP

#include <algorithm>

#include "teleop_msgs/msg/gamepad_state.hpp"

namespace teleop {

struct WheelSpeeds {
    double left;
    double right;

    WheelSpeeds(const double& left, const double& right)
    : left(std::clamp(left, -1.0, 1.0)), right(std::clamp(right, -1.0, 1.0)) {

    }
};

class DriveControlStrategy {
public:

    virtual WheelSpeeds operator()(teleop_msgs::msg::GamepadState state) = 0;

};

}  /* namespace teleop */

#endif