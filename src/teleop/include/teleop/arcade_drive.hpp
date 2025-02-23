#ifndef TELEOP_ARCADE_DRIVE_HPP
#define TELEOP_ARCADE_DRIVE_HPP

#include "teleop/control.hpp"
#include "teleop/signal_processing.hpp"

namespace teleop {

class ArcadeDrive: public DriveControlStrategy {

    bool _invert_linear;
    bool _invert_turn;
    bool _square_inputs;

    Deadband _deadband;

public:

    ArcadeDrive()
    : _invert_linear(false), _invert_turn(false), _square_inputs(true) {}

    ArcadeDrive(const bool& invert_linear, const bool& invert_turn, const bool& square_inputs)
    : _invert_linear(invert_linear), _invert_turn(invert_turn), _square_inputs(square_inputs) {}



    const bool& invert_linear() const { return _invert_linear; }
    bool& invert_linear() { return _invert_linear; }
    const bool& invert_turn() const { return _invert_turn; }
    bool& invert_turn() { return _invert_turn; }

    virtual WheelSpeeds operator()(GamepadState gamepad_state);

};

}  /* namespace teleop */

#endif