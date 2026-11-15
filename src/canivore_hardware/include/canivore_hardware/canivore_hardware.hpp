#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/TalonFXS.hpp>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "serial_msgs/msg/current_bus_voltage.hpp"
#include "serial_msgs/msg/position.hpp"
#include "serial_msgs/msg/temperature.hpp"

namespace canivore_hardware
{

/// talonfx: Kraken X60 (wheels, drums), closed-loop velocity in rad/s.
/// talonfxs: Talon FXS driving a brushed linear actuator (arms), open-loop duty cycle.
enum class DeviceType { TalonFX, TalonFXS };

/// Position sensor on a Talon FXS actuator.
enum class ActuatorSensor { None, Analog, Quadrature, PulseWidth };

/// Everything the plugin knows about one ros2_control joint.
struct JointMotor
{
  // ---- Common params (<joint><param .../></joint>) ----
  std::string name;
  DeviceType device_type{DeviceType::TalonFX};
  int can_id{-1};                 // -1 = not connected: the joint echoes its command
  bool inverted{false};           // flip so that +command matches the URDF convention
  bool brake{true};               // neutral mode: brake (true) or coast (false)
  // talonfx: clamp in rad/s (0 = none). talonfxs: the command that means 100% output.
  double max_velocity{0.0};
  double stator_current_limit{0.0};  // A, 0 = keep Phoenix default
  double supply_current_limit{0.0};  // A, 0 = keep Phoenix default
  std::optional<double> min_position;  // rad
  std::optional<double> max_position;  // rad

  // ---- talonfx only ----
  double gear_ratio{1.0};         // motor rotations per joint rotation
  double kP{0.0}, kS{0.0}, kV{0.0};  // Slot0 gains, in Phoenix units (V per rotation/s)
  std::optional<double> initial_position;  // rad, encoder is set to this on configure

  // ---- talonfxs only (linear actuators) ----
  // Which Talon FXS output leads the actuator is wired to (A+B, A+C or B+C).
  ctre::phoenix6::signals::BrushedMotorWiringValue brushed_wiring{
    ctre::phoenix6::signals::BrushedMotorWiringValue::Leads_A_and_B};
  ActuatorSensor sensor{ActuatorSensor::None};
  int quadrature_edges_per_rotation{4096};
  // Raw sensor readings (volts for analog, rotations for quadrature/pulse width) with the
  // actuator fully retracted and fully extended. Needed to measure extension.
  std::optional<double> sensor_retracted;
  std::optional<double> sensor_extended;

  // ---- Arm joints: angle <-> 0..1 actuator extension (same as the sim's actuator_feedback.yaml).
  // Used to publish /position and to turn a measured extension into the arm angle.
  double feedback_angle_start{-1.2}, feedback_angle_end{0.2};
  double feedback_output_start{1.0}, feedback_output_end{0.0};

  // ---- Interface names, built once so read()/write() don't rebuild strings ----
  std::string cmd_velocity, st_position, st_velocity, st_effort;
  bool has_position{false}, has_velocity{false}, has_effort{false};

  // ---- Phoenix objects, only created for connected joints ----
  std::unique_ptr<ctre::phoenix6::hardware::TalonFX> talon_fx;
  std::unique_ptr<ctre::phoenix6::hardware::TalonFXS> talon_fxs;
  ctre::phoenix6::controls::VelocityVoltage velocity_request{units::turns_per_second_t{0.0}};
  ctre::phoenix6::controls::DutyCycleOut duty_cycle_request{0.0};
  // Status signals (owned by the device objects).
  ctre::phoenix6::StatusSignal<units::angle::turn_t> * position_signal{nullptr};
  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> * velocity_signal{nullptr};
  ctre::phoenix6::StatusSignal<units::current::ampere_t> * current_signal{nullptr};
  ctre::phoenix6::StatusSignal<units::temperature::celsius_t> * temperature_signal{nullptr};
  ctre::phoenix6::StatusSignal<units::voltage::volt_t> * supply_signal{nullptr};
  ctre::phoenix6::StatusSignal<units::voltage::volt_t> * analog_signal{nullptr};

  // ---- Latest values ----
  double position{0.0};    // rad
  double velocity{0.0};    // rad/s
  double current{0.0};     // A (stator current, used as effort)
  double temperature{0.0}; // degrees C
  double supply_voltage{0.0};
  std::optional<double> extension;  // 0..1 actuator extension, when measured by a sensor

  bool connected() const {return can_id >= 0;}
  bool has_hardware() const {return talon_fx || talon_fxs;}
  bool measures_extension() const
  {
    return device_type == DeviceType::TalonFXS && sensor != ActuatorSensor::None &&
           sensor_retracted && sensor_extended && *sensor_retracted != *sensor_extended;
  }
  /// Arm angle (rad) -> 0..1 extension, and back, using the feedback_* mapping.
  double angle_to_extension(double angle) const
  {
    const double fraction = (angle - feedback_angle_start) / (feedback_angle_end - feedback_angle_start);
    return feedback_output_start + fraction * (feedback_output_end - feedback_output_start);
  }
  double extension_to_angle(double ext) const
  {
    const double fraction = (ext - feedback_output_start) / (feedback_output_end - feedback_output_start);
    return feedback_angle_start + fraction * (feedback_angle_end - feedback_angle_start);
  }
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
  bool configure_talon_fx(JointMotor & motor);
  bool configure_talon_fxs(JointMotor & motor);
  template<typename Device>
  void collect_signals(JointMotor & motor, Device & device);
  void read_motor(JointMotor & motor, double period_s);
  void stop_all_motors();
  void publish_feedback(const rclcpp::Time & time);
  void publish_position(const rclcpp::Time & time);
  JointMotor * find(const std::string & joint_name);

  // ---- Hardware-level params (<hardware><param .../></hardware>) ----
  std::string canbus_name_{"*"};  // must outlive canbus_, which keeps a view of it
  int feed_timeout_ms_{100};
  bool enable_foc_{false};
  double status_frequency_hz_{100.0};
  double feedback_publish_hz_{10.0};
  double position_publish_hz_{50.0};
  double activation_timeout_s_{0.5};

  std::unique_ptr<ctre::phoenix6::CANBus> canbus_;
  std::vector<JointMotor> motors_;
  // Every status signal of every connected device, refreshed together once per read().
  std::vector<ctre::phoenix6::BaseStatusSignal *> signals_;
  ctre::phoenix6::controls::NeutralOut neutral_request_{};

  rclcpp::Publisher<serial_msgs::msg::CurrentBusVoltage>::SharedPtr current_bus_pub_;
  rclcpp::Publisher<serial_msgs::msg::Temperature>::SharedPtr temperature_pub_;
  rclcpp::Publisher<serial_msgs::msg::Position>::SharedPtr position_pub_;
  double last_feedback_publish_s_{-1.0e9};  // seconds; plain double avoids clock-type mismatches
  double last_position_publish_s_{-1.0e9};
  bool read_error_logged_{false};
};

}  // namespace canivore_hardware
