#ifndef TELEOP_RAMP_HPP
#define TELEOP_RAMP_HPP

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
    : _falling_ramp_rate(-abs(ramp_rate)), _rising_ramp_rate(abs(ramp_rate)), clock(clock) {
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

}  /* namespace teleop */

#endif