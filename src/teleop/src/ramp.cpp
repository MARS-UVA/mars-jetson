#include "teleop/ramp.hpp"

namespace teleop {

double Ramp::operator()(double signal) {
    double output;
    rclcpp::Time this_time = clock.now();
    if (!(last_time.has_value() && last_output.has_value())) {
        output = signal;
    } else {
        double dt = (this_time - last_time.value()).seconds();
        double input_slope = (signal - last_output.value()) / dt;
        double output_slope;
        if (input_slope < _falling_ramp_rate) {
            output_slope = _falling_ramp_rate;
        } else if (output_slope > _rising_ramp_rate) {
            output_slope = _rising_ramp_rate;
        } else {
            output_slope = input_slope;
        }
        output = last_output.value() + (output_slope * dt);
    }
    last_output = output;
    last_time = this_time;
    return output;
}

void Ramp::reset() {
    last_time = std::nullopt;
    last_output = std::nullopt;
}

}  /* namespace teleop */
