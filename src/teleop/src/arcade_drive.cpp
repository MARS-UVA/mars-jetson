#include <algorithm>
#include <cmath>

#include "teleop/arcade_drive.hpp"
#include "teleop/control.hpp"

namespace teleop {

WheelSpeeds ArcadeDrive::operator()(GamepadState gamepad_state) {
    double linear_rate = (_invert_linear ? -1 : 1) * std::clamp(gamepad_state.left_stick.y, -1.0, 1.0);
    double turn_rate = (_invert_turn ? -1 : 1) * std::clamp(gamepad_state.left_stick.x, -1.0, 1.0);

    if (_square_inputs) {
        linear_rate = std::copysign(linear_rate * linear_rate, linear_rate);
        turn_rate = std::copysign(turn_rate * turn_rate, turn_rate);
    }

    double left_speed = linear_rate - turn_rate;
    double right_speed = linear_rate + turn_rate;

    double greater_input = std::max(std::fabs(linear_rate), std::fabs(turn_rate));
    double lesser_input = std::min(std::fabs(linear_rate), std::fabs(turn_rate));

    if (greater_input == 0.0) {
        return { 0.0, 0.0 };
    }
    double saturated_input = (greater_input + lesser_input) / greater_input;
    left_speed /= saturated_input;
    right_speed /= saturated_input;

    return { left_speed, right_speed };
}

}  /* namespace teleop */