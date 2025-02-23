#ifndef TELEOP_SIGNAL_PROCESSING_HPP
#define TELEOP_SIGNAL_PROCESSING_HPP

#include <cmath>
#include <exception>
#include <optional>

#include "rclcpp/clock.hpp"

namespace teleop {

class Ramp {

    double _falling_ramp_rate;
    double _rising_ramp_rate;
    rclcpp::Clock& clock;

    std::optional<rclcpp::Time> last_time = std::nullopt;
    std::optional<double> last_output = std::nullopt;

public:

    Ramp(const double& ramp_rate, rclcpp::Clock& clock) noexcept(false)
    : _falling_ramp_rate(-std::fabs(ramp_rate)), _rising_ramp_rate(std::fabs(ramp_rate)), clock(clock) {
        check_valid_ramp_rate();
    }

    Ramp(const double& falling_ramp_rate, const double& rising_ramp_rate, rclcpp::Clock& clock) noexcept(false)
    : _falling_ramp_rate(falling_ramp_rate), _rising_ramp_rate(rising_ramp_rate), clock(clock) {
        check_valid_ramp_rate();
    }

    const double& falling_ramp_rate() const { return _falling_ramp_rate; }
    double& falling_ramp_rate() { return _falling_ramp_rate; }
    const double& rising_ramp_rate() const { return _rising_ramp_rate; }
    double& rising_ramp_rate() { return _rising_ramp_rate; }

    double operator()(double signal);

    void reset();

private:

    void check_valid_ramp_rate() const noexcept(false) {
        if (_falling_ramp_rate >= 0) {
            throw std::logic_error("falling_ramp_rate must be negative");
        } else if (_rising_ramp_rate <= 0) {
            throw std::logic_error("rising_ramp_rate must be positive");
        }
    }

};

class Deadband {

    double _min_magnitude;

public:

    Deadband()
    : _min_magnitude(0) {

    }

    Deadband(double min_magnitude)
    : _min_magnitude(abs(min_magnitude)) {

    }

    const double& min_magnitude() const { return _min_magnitude; }
    double& min_magnitude() { return _min_magnitude; }

    double operator()(double signal) const;

};

}  /* namespace teleop */

#endif