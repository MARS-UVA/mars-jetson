#ifndef TELEOP_CONTROL_HPP
#define TELEOP_CONTROL_HPP

#include <algorithm>

namespace teleop {

struct WheelSpeeds {
    double left;
    double right;

    WheelSpeeds(const double& left, const double& right)
    : left(std::clamp(left, -1.0, 1.0)), right(std::clamp(right, -1.0, 1.0)) {

    }
};

struct GamepadState {
    StickPosition left_stick;
    StickPosition right_stick;
};

struct StickPosition {
    double x;
    double y;
};

class DriveControlStrategy {
public:

    virtual WheelSpeeds operator()(GamepadState gamepad_state) = 0;

};

}  /* namespace teleop */

#endif