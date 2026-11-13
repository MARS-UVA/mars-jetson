#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <ctre/phoenix6/TalonFX.hpp>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "serial_msgs/msg/current_bus_voltage.hpp"
#include "serial_msgs/msg/temperature.hpp"

namespace canivore_hardware
{

/// Everything the plugin knows about one ros2_control joint.
struct JointMotor
{
  // ---- Read from the URDF (<joint><param .../></joint>) ----
  std::string name;
  int can_id{-1};                 // -1 = not connected: the joint echoes its command
  double gear_ratio{1.0};         // motor rotations per joint rotation
  bool inverted{false};           // flip so that +velocity matches the URDF convention
  bool brake{true};               // neutral mode: brake (true) or coast (false)
  double max_velocity{0.0};       // rad/s clamp on commands, 0 = no clamp
  double stator_current_limit{0.0};  // A, 0 = keep Phoenix default
  double supply_current_limit{0.0};  // A, 0 = keep Phoenix default
  double kP{0.0}, kS{0.0}, kV{0.0};  // Slot0 gains, in Phoenix units (V per rotation/s)
  std::optional<double> min_position;      // rad, enables reverse soft limit
  std::optional<double> max_position;      // rad, enables forward soft limit
  std::optional<double> initial_position;  // rad, encoder is set to this on configure

  // ---- Interface names, built once so read()/write() don't rebuild strings ----
  std::string cmd_velocity, st_position, st_velocity, st_effort;
  bool has_position{false}, has_velocity{false}, has_effort{false};

  // ---- Phoenix objects, only created for connected joints ----
  std::unique_ptr<ctre::phoenix6::hardware::TalonFX> talon;
  ctre::phoenix6::controls::VelocityVoltage velocity_request{units::turns_per_second_t{0.0}};

  // ---- Latest values ----
  double position{0.0};    // rad
  double velocity{0.0};    // rad/s
  double current{0.0};     // A (stator current, used as effort)
  double temperature{0.0}; // degrees C
  double supply_voltage{0.0};

  bool connected() const {return can_id >= 0;}
};

class CanivoreHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool configure_motor(JointMotor & motor);
  void stop_all_motors();
  void publish_feedback(const rclcpp::Time & time);
  JointMotor * find(const std::string & joint_name);

  // ---- Hardware-level params (<hardware><param .../></hardware>) ----
  std::string canbus_name_{"*"};  // must outlive canbus_, which keeps a view of it
  int feed_timeout_ms_{100};
  bool enable_foc_{false};
  double status_frequency_hz_{100.0};
  double feedback_publish_hz_{10.0};
  double activation_timeout_s_{0.5};

  std::unique_ptr<ctre::phoenix6::CANBus> canbus_;
  std::vector<JointMotor> motors_;
  // Every status signal of every connected motor, refreshed together once per read().
  std::vector<ctre::phoenix6::BaseStatusSignal *> signals_;
  ctre::phoenix6::controls::NeutralOut neutral_request_{};

  rclcpp::Publisher<serial_msgs::msg::CurrentBusVoltage>::SharedPtr current_bus_pub_;
  rclcpp::Publisher<serial_msgs::msg::Temperature>::SharedPtr temperature_pub_;
  double last_feedback_publish_s_{-1.0e9};  // seconds; plain double avoids clock-type mismatches
  bool read_error_logged_{false};
};

}  // namespace canivore_hardware
