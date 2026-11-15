#include "canivore_hardware/canivore_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <set>
#include <stdexcept>

#include <ctre/phoenix6/unmanaged/Unmanaged.hpp>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
namespace p6 = ctre::phoenix6;

namespace canivore_hardware
{

namespace
{

constexpr double kTwoPi = 2.0 * std::numbers::pi;

double rad_to_turns(double rad) {return rad / kTwoPi;}
double turns_to_rad(double turns) {return turns * kTwoPi;}

// ---- Parameter helpers. URDF params are always strings; these parse them with defaults. ----

using ParamMap = std::unordered_map<std::string, std::string>;

const std::string * lookup(const ParamMap & params, const std::string & key)
{
  const auto it = params.find(key);
  if (it == params.end() || it->second.empty()) {
    return nullptr;
  }
  return &it->second;
}

std::string get_string(const ParamMap & params, const std::string & key, const std::string & fallback)
{
  const auto * value = lookup(params, key);
  return value ? *value : fallback;
}

double get_double(const ParamMap & params, const std::string & key, double fallback)
{
  const auto * value = lookup(params, key);
  if (!value) {
    return fallback;
  }
  try {
    return std::stod(*value);
  } catch (const std::exception &) {
    throw std::invalid_argument("param '" + key + "' is not a number: '" + *value + "'");
  }
}

std::optional<double> get_optional_double(const ParamMap & params, const std::string & key)
{
  // xacro writes "None" for values that were left out of the YAML.
  const auto * value = lookup(params, key);
  if (!value || *value == "None" || *value == "none") {
    return std::nullopt;
  }
  return get_double(params, key, 0.0);
}

int get_int(const ParamMap & params, const std::string & key, int fallback)
{
  return static_cast<int>(std::lround(get_double(params, key, fallback)));
}

bool get_bool(const ParamMap & params, const std::string & key, bool fallback)
{
  const auto * value = lookup(params, key);
  if (!value) {
    return fallback;
  }
  std::string v = *value;
  std::transform(v.begin(), v.end(), v.begin(), ::tolower);
  if (v == "true" || v == "1" || v == "yes") {return true;}
  if (v == "false" || v == "0" || v == "no") {return false;}
  throw std::invalid_argument("param '" + key + "' is not a boolean: '" + *value + "'");
}

}  // namespace

// =====================================================================================
// on_init: read and validate parameters. Never touches the CAN bus.
// =====================================================================================
CallbackReturn CanivoreHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  const auto & info = get_hardware_info();

  try {
    const auto & hw = info.hardware_parameters;
    canbus_name_ = get_string(hw, "canbus", "*");
    feed_timeout_ms_ = get_int(hw, "feed_timeout_ms", 100);
    enable_foc_ = get_bool(hw, "enable_foc", false);
    status_frequency_hz_ = get_double(hw, "status_frequency_hz", 100.0);
    feedback_publish_hz_ = get_double(hw, "feedback_publish_hz", 10.0);
    position_publish_hz_ = get_double(hw, "position_publish_hz", 50.0);
    activation_timeout_s_ = get_double(hw, "activation_timeout_s", 0.5);

    std::set<int> used_ids;
    for (const auto & joint : info.joints) {
      JointMotor m;
      m.name = joint.name;
      const auto & jp = joint.parameters;

      const auto type = get_string(jp, "device_type", "talonfx");
      if (type == "talonfx") {
        m.device_type = DeviceType::TalonFX;
      } else if (type == "talonfxs") {
        m.device_type = DeviceType::TalonFXS;
      } else {
        RCLCPP_ERROR(get_logger(), "Joint '%s': device_type must be talonfx or talonfxs, not '%s'",
          m.name.c_str(), type.c_str());
        return CallbackReturn::ERROR;
      }
      const bool fxs = m.device_type == DeviceType::TalonFXS;

      m.can_id = get_int(jp, "can_id", -1);
      m.inverted = get_bool(jp, "inverted", false);
      m.brake = get_string(jp, "neutral_mode", "brake") != "coast";
      m.max_velocity = get_double(jp, "max_velocity", fxs ? 1.0 : 0.0);
      m.stator_current_limit = get_double(jp, "stator_current_limit", 0.0);
      m.supply_current_limit = get_double(jp, "supply_current_limit", 0.0);
      m.min_position = get_optional_double(jp, "min_position");
      m.max_position = get_optional_double(jp, "max_position");

      // talonfx
      m.gear_ratio = get_double(jp, "gear_ratio", 1.0);
      m.kP = get_double(jp, "kP", 0.0);
      m.kS = get_double(jp, "kS", 0.0);
      m.kV = get_double(jp, "kV", 0.0);
      m.initial_position = get_optional_double(jp, "initial_position");

      // talonfxs
      const auto wiring = get_string(jp, "brushed_wiring", "AB");
      if (wiring == "AB") {
        m.brushed_wiring = p6::signals::BrushedMotorWiringValue::Leads_A_and_B;
      } else if (wiring == "AC") {
        m.brushed_wiring = p6::signals::BrushedMotorWiringValue::Leads_A_and_C;
      } else if (wiring == "BC") {
        m.brushed_wiring = p6::signals::BrushedMotorWiringValue::Leads_B_and_C;
      } else {
        RCLCPP_ERROR(get_logger(), "Joint '%s': brushed_wiring must be AB, AC or BC, not '%s'",
          m.name.c_str(), wiring.c_str());
        return CallbackReturn::ERROR;
      }
      const auto sensor = get_string(jp, "feedback_sensor", "none");
      if (sensor == "none" || sensor == "None") {
        m.sensor = ActuatorSensor::None;
      } else if (sensor == "analog") {
        m.sensor = ActuatorSensor::Analog;
      } else if (sensor == "quadrature") {
        m.sensor = ActuatorSensor::Quadrature;
      } else if (sensor == "pulsewidth") {
        m.sensor = ActuatorSensor::PulseWidth;
      } else {
        RCLCPP_ERROR(get_logger(),
          "Joint '%s': feedback_sensor must be none, analog, quadrature or pulsewidth, not '%s'",
          m.name.c_str(), sensor.c_str());
        return CallbackReturn::ERROR;
      }
      m.quadrature_edges_per_rotation = get_int(jp, "quadrature_edges_per_rotation", 4096);
      m.sensor_retracted = get_optional_double(jp, "sensor_retracted");
      m.sensor_extended = get_optional_double(jp, "sensor_extended");

      // arm angle <-> extension mapping
      m.feedback_angle_start = get_double(jp, "feedback_angle_start", -1.2);
      m.feedback_angle_end = get_double(jp, "feedback_angle_end", 0.2);
      m.feedback_output_start = get_double(jp, "feedback_output_start", 1.0);
      m.feedback_output_end = get_double(jp, "feedback_output_end", 0.0);
      if (m.feedback_angle_start == m.feedback_angle_end ||
        m.feedback_output_start == m.feedback_output_end)
      {
        RCLCPP_ERROR(get_logger(), "Joint '%s': feedback_angle_start/end and "
          "feedback_output_start/end must each differ", m.name.c_str());
        return CallbackReturn::ERROR;
      }

      if (m.connected()) {
        if (m.can_id > 62) {
          RCLCPP_ERROR(get_logger(), "Joint '%s': CAN ID %d is out of range 0-62",
            m.name.c_str(), m.can_id);
          return CallbackReturn::ERROR;
        }
        if (!used_ids.insert(m.can_id).second) {
          RCLCPP_ERROR(get_logger(), "Joint '%s': CAN ID %d is used by another joint",
            m.name.c_str(), m.can_id);
          return CallbackReturn::ERROR;
        }
        if (!fxs && (m.min_position || m.max_position) && !m.initial_position) {
          RCLCPP_WARN(get_logger(),
            "Joint '%s': soft limits are set but initial_position is not. The encoder will read 0 "
            "wherever the joint is at power-on, so the limits will be in the wrong place.",
            m.name.c_str());
        }
        if (fxs && !m.measures_extension()) {
          RCLCPP_WARN(get_logger(),
            "Joint '%s': no calibrated actuator sensor (feedback_sensor, sensor_retracted, "
            "sensor_extended). Its position will be ESTIMATED from commands and the plugin "
            "cannot stop it at the ends of travel; the actuator's own limit switches must.",
            m.name.c_str());
        }
      }
      if (m.gear_ratio <= 0.0) {
        RCLCPP_ERROR(get_logger(), "Joint '%s': gear_ratio must be > 0", m.name.c_str());
        return CallbackReturn::ERROR;
      }
      if (fxs && m.max_velocity <= 0.0) {
        RCLCPP_ERROR(get_logger(), "Joint '%s': a talonfxs joint needs max_velocity > 0 "
          "(the command that means 100%% output)", m.name.c_str());
        return CallbackReturn::ERROR;
      }

      // This plugin only accepts velocity commands, which is what both controllers send.
      bool has_velocity_command = false;
      for (const auto & ci : joint.command_interfaces) {
        if (ci.name == hardware_interface::HW_IF_VELOCITY) {
          has_velocity_command = true;
        } else {
          RCLCPP_ERROR(get_logger(), "Joint '%s': unsupported command interface '%s'",
            m.name.c_str(), ci.name.c_str());
          return CallbackReturn::ERROR;
        }
      }
      if (!has_velocity_command) {
        RCLCPP_ERROR(get_logger(), "Joint '%s' needs a velocity command interface",
          m.name.c_str());
        return CallbackReturn::ERROR;
      }
      for (const auto & si : joint.state_interfaces) {
        if (si.name == hardware_interface::HW_IF_POSITION) {m.has_position = true;}
        if (si.name == hardware_interface::HW_IF_VELOCITY) {m.has_velocity = true;}
        if (si.name == hardware_interface::HW_IF_EFFORT) {m.has_effort = true;}
      }

      m.cmd_velocity = m.name + "/" + hardware_interface::HW_IF_VELOCITY;
      m.st_position = m.name + "/" + hardware_interface::HW_IF_POSITION;
      m.st_velocity = m.name + "/" + hardware_interface::HW_IF_VELOCITY;
      m.st_effort = m.name + "/" + hardware_interface::HW_IF_EFFORT;
      m.position = m.initial_position.value_or(0.0);

      const std::string device = fxs ? "Talon FXS actuator" : "TalonFX";
      RCLCPP_INFO(get_logger(), "Joint '%s': %s%s",
        m.name.c_str(),
        m.connected() ? (device + " CAN ID " + std::to_string(m.can_id)).c_str() :
        "NOT CONNECTED (echoes commands)",
        m.inverted ? ", inverted" : "");
      motors_.push_back(std::move(m));
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Bad hardware parameter: %s", e.what());
    return CallbackReturn::ERROR;
  }

  // Same feedback topics the serial plugin publishes, so the dashboard and dig/dump keep working.
  if (auto node = get_node()) {
    position_pub_ = node->create_publisher<serial_msgs::msg::Position>("position", 10);
    current_bus_pub_ =
      node->create_publisher<serial_msgs::msg::CurrentBusVoltage>("current_bus_voltage", 10);
    temperature_pub_ = node->create_publisher<serial_msgs::msg::Temperature>("temperature", 10);
  } else {
    RCLCPP_WARN(get_logger(), "No node available; feedback topics will not be published");
  }

  RCLCPP_INFO(get_logger(), "Configured %zu joints on CAN bus '%s'", motors_.size(),
    canbus_name_.c_str());
  return CallbackReturn::SUCCESS;
}

// =====================================================================================
// on_configure: create the device objects and push configs to them. Apply() blocks, which is
// fine here but would not be in read()/write().
// =====================================================================================
template<typename Device>
void CanivoreHardware::collect_signals(JointMotor & m, Device & device)
{
  // Pointers stay valid for the lifetime of the device object.
  m.position_signal = &device.GetPosition(false);
  m.velocity_signal = &device.GetVelocity(false);
  m.current_signal = &device.GetStatorCurrent(false);
  m.temperature_signal = &device.GetDeviceTemp(false);
  m.supply_signal = &device.GetSupplyVoltage(false);
  signals_.insert(signals_.end(),
    {m.position_signal, m.velocity_signal, m.current_signal, m.temperature_signal, m.supply_signal});
}

CallbackReturn CanivoreHardware::on_configure(const rclcpp_lifecycle::State &)
{
  signals_.clear();
  const bool any_connected =
    std::any_of(motors_.begin(), motors_.end(), [](const auto & m) {return m.connected();});
  if (!any_connected) {
    RCLCPP_WARN(get_logger(), "No joints have a CAN ID; running with every joint echoing commands");
    return CallbackReturn::SUCCESS;
  }

  canbus_ = std::make_unique<p6::CANBus>(canbus_name_);
  for (auto & m : motors_) {
    if (!m.connected()) {
      continue;
    }
    if (m.device_type == DeviceType::TalonFX) {
      m.talon_fx = std::make_unique<p6::hardware::TalonFX>(m.can_id, *canbus_);
      if (!configure_talon_fx(m)) {
        return CallbackReturn::ERROR;
      }
      collect_signals(m, *m.talon_fx);
    } else {
      m.talon_fxs = std::make_unique<p6::hardware::TalonFXS>(m.can_id, *canbus_);
      if (!configure_talon_fxs(m)) {
        return CallbackReturn::ERROR;
      }
      collect_signals(m, *m.talon_fxs);
      if (m.sensor == ActuatorSensor::Analog) {
        m.analog_signal = &m.talon_fxs->GetAnalogVoltage(false);
        signals_.push_back(m.analog_signal);
      }
    }
  }

  // Ask for the signals we use at a steady rate, then turn off every status frame we don't use.
  p6::BaseStatusSignal::SetUpdateFrequencyForAll(units::hertz_t{status_frequency_hz_}, signals_);
  for (auto & m : motors_) {
    if (m.talon_fx) {
      m.talon_fx->OptimizeBusUtilization();
    }
    if (m.talon_fxs) {
      m.talon_fxs->OptimizeBusUtilization();
    }
  }
  return CallbackReturn::SUCCESS;
}

bool CanivoreHardware::configure_talon_fx(JointMotor & m)
{
  p6::configs::TalonFXConfiguration config{};

  config.MotorOutput.NeutralMode =
    m.brake ? p6::signals::NeutralModeValue::Brake : p6::signals::NeutralModeValue::Coast;
  config.MotorOutput.Inverted = m.inverted ?
    p6::signals::InvertedValue::Clockwise_Positive :
    p6::signals::InvertedValue::CounterClockwise_Positive;

  // With this set, every position/velocity the TalonFX reports or accepts is in *joint*
  // rotations, so the code only converts rotations <-> radians.
  config.Feedback.SensorToMechanismRatio = m.gear_ratio;

  if (m.stator_current_limit > 0.0) {
    config.CurrentLimits.StatorCurrentLimit = units::ampere_t{m.stator_current_limit};
    config.CurrentLimits.StatorCurrentLimitEnable = true;
  }
  if (m.supply_current_limit > 0.0) {
    config.CurrentLimits.SupplyCurrentLimit = units::ampere_t{m.supply_current_limit};
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
  }

  config.Slot0.kP = m.kP;
  config.Slot0.kS = m.kS;
  config.Slot0.kV = m.kV;

  if (m.max_position) {
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = units::turn_t{rad_to_turns(*m.max_position)};
  }
  if (m.min_position) {
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = units::turn_t{rad_to_turns(*m.min_position)};
  }

  const auto status = m.talon_fx->GetConfigurator().Apply(config);
  if (!status.IsOK()) {
    RCLCPP_ERROR(get_logger(),
      "Joint '%s': could not configure TalonFX CAN ID %d on bus '%s': %s. "
      "Check the CAN ID, the bus name and that the device is powered.",
      m.name.c_str(), m.can_id, canbus_name_.c_str(), status.GetName());
    return false;
  }

  // The built-in encoder starts at zero wherever the joint is at power-on. If the joint has a
  // known starting position, tell the encoder.
  if (m.initial_position) {
    const auto pos_status = m.talon_fx->SetPosition(units::turn_t{rad_to_turns(*m.initial_position)});
    if (!pos_status.IsOK()) {
      RCLCPP_ERROR(get_logger(), "Joint '%s': could not set initial position: %s",
        m.name.c_str(), pos_status.GetName());
      return false;
    }
  }
  return true;
}

bool CanivoreHardware::configure_talon_fxs(JointMotor & m)
{
  p6::configs::TalonFXSConfiguration config{};

  // A Talon FXS drives nothing until told what is connected to it.
  config.Commutation.MotorArrangement = p6::signals::MotorArrangementValue::Brushed_DC;
  config.Commutation.BrushedMotorWiring = m.brushed_wiring;

  config.MotorOutput.NeutralMode =
    m.brake ? p6::signals::NeutralModeValue::Brake : p6::signals::NeutralModeValue::Coast;
  config.MotorOutput.Inverted = m.inverted ?
    p6::signals::InvertedValue::Clockwise_Positive :
    p6::signals::InvertedValue::CounterClockwise_Positive;

  if (m.stator_current_limit > 0.0) {
    config.CurrentLimits.StatorCurrentLimit = units::ampere_t{m.stator_current_limit};
    config.CurrentLimits.StatorCurrentLimitEnable = true;
  }
  if (m.supply_current_limit > 0.0) {
    config.CurrentLimits.SupplyCurrentLimit = units::ampere_t{m.supply_current_limit};
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
  }

  // Encoder-type actuator sensors feed the FXS's position signal; an analog potentiometer is
  // read separately as a voltage.
  if (m.sensor == ActuatorSensor::Quadrature) {
    config.ExternalFeedback.ExternalFeedbackSensorSource =
      p6::signals::ExternalFeedbackSensorSourceValue::Quadrature;
    config.ExternalFeedback.QuadratureEdgesPerRotation = m.quadrature_edges_per_rotation;
  } else if (m.sensor == ActuatorSensor::PulseWidth) {
    config.ExternalFeedback.ExternalFeedbackSensorSource =
      p6::signals::ExternalFeedbackSensorSourceValue::PulseWidth;
  }

  const auto status = m.talon_fxs->GetConfigurator().Apply(config);
  if (!status.IsOK()) {
    RCLCPP_ERROR(get_logger(),
      "Joint '%s': could not configure Talon FXS CAN ID %d on bus '%s': %s. "
      "Check the CAN ID, the bus name and that the device is powered.",
      m.name.c_str(), m.can_id, canbus_name_.c_str(), status.GetName());
    return false;
  }
  return true;
}

CallbackReturn CanivoreHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  stop_all_motors();
  signals_.clear();
  for (auto & m : motors_) {
    m.position_signal = nullptr;
    m.velocity_signal = nullptr;
    m.current_signal = nullptr;
    m.temperature_signal = nullptr;
    m.supply_signal = nullptr;
    m.analog_signal = nullptr;
    m.talon_fx.reset();
    m.talon_fxs.reset();
  }
  canbus_.reset();
  return CallbackReturn::SUCCESS;
}

// =====================================================================================
// on_activate: make sure every connected device is actually answering before anything moves.
// =====================================================================================
CallbackReturn CanivoreHardware::on_activate(const rclcpp_lifecycle::State &)
{
  if (!signals_.empty()) {
    p6::BaseStatusSignal::WaitForAll(units::second_t{activation_timeout_s_}, false, signals_);
    bool all_ok = true;
    for (auto & m : motors_) {
      if (!m.has_hardware()) {
        continue;
      }
      const auto status = m.position_signal->GetStatus();
      if (!status.IsOK()) {
        RCLCPP_ERROR(get_logger(), "Joint '%s': %s CAN ID %d not responding on bus '%s': %s",
          m.name.c_str(), m.talon_fx ? "TalonFX" : "Talon FXS", m.can_id, canbus_name_.c_str(),
          status.GetName());
        all_ok = false;
      }
    }
    if (!all_ok) {
      return CallbackReturn::ERROR;
    }
  }

  // Start with zero commands and states that reflect the hardware.
  for (auto & m : motors_) {
    set_command(m.cmd_velocity, 0.0);
    if (m.has_hardware()) {
      read_motor(m, 0.0);
    }
    m.velocity = 0.0;
    if (m.has_position) {set_state(m.st_position, m.position);}
    if (m.has_velocity) {set_state(m.st_velocity, 0.0);}
    if (m.has_effort) {set_state(m.st_effort, 0.0);}
  }
  read_error_logged_ = false;
  RCLCPP_INFO(get_logger(), "Activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn CanivoreHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  stop_all_motors();
  RCLCPP_INFO(get_logger(), "Deactivated, motors set to neutral");
  return CallbackReturn::SUCCESS;
}

CallbackReturn CanivoreHardware::on_shutdown(const rclcpp_lifecycle::State &)
{
  stop_all_motors();
  return CallbackReturn::SUCCESS;
}

CallbackReturn CanivoreHardware::on_error(const rclcpp_lifecycle::State &)
{
  stop_all_motors();
  return CallbackReturn::SUCCESS;
}

void CanivoreHardware::stop_all_motors()
{
  for (auto & m : motors_) {
    if (m.talon_fx) {
      m.talon_fx->SetControl(neutral_request_);
    }
    if (m.talon_fxs) {
      m.talon_fxs->SetControl(neutral_request_);
    }
  }
}

// Turn one device's latest signal values into joint state. Signals must already be refreshed.
void CanivoreHardware::read_motor(JointMotor & m, double period_s)
{
  m.current = m.current_signal->GetValueAsDouble();
  m.temperature = m.temperature_signal->GetValueAsDouble();
  m.supply_voltage = m.supply_signal->GetValueAsDouble();

  if (m.talon_fx) {
    m.position = turns_to_rad(m.position_signal->GetValueAsDouble());
    m.velocity = turns_to_rad(m.velocity_signal->GetValueAsDouble());
    return;
  }

  // Talon FXS actuator
  if (!m.measures_extension()) {
    m.extension.reset();
    return;  // estimated from commands in read()
  }
  const double raw = m.sensor == ActuatorSensor::Analog ?
    m.analog_signal->GetValueAsDouble() : m.position_signal->GetValueAsDouble();
  const double ext = (raw - *m.sensor_retracted) / (*m.sensor_extended - *m.sensor_retracted);
  const double angle = m.extension_to_angle(ext);
  if (period_s > 0.0) {
    // The actuator reports extension, not arm speed, so differentiate the angle (lightly
    // smoothed; a potentiometer reading is noisy).
    const double raw_velocity = (angle - m.position) / period_s;
    m.velocity = 0.8 * m.velocity + 0.2 * raw_velocity;
  }
  m.extension = ext;
  m.position = angle;
}

// =====================================================================================
// read(): runs every control cycle (500 Hz in controllers.yaml).
// =====================================================================================
return_type CanivoreHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (!signals_.empty()) {
    // One call refreshes every signal from every device; much cheaper than one call each.
    const auto status = p6::BaseStatusSignal::RefreshAll(false, signals_);
    if (!status.IsOK()) {
      if (!read_error_logged_) {
        RCLCPP_ERROR(get_logger(), "Lost contact with motors on bus '%s': %s",
          canbus_name_.c_str(), status.GetName());
        read_error_logged_ = true;
      }
      // Returning ERROR makes the controller manager deactivate the hardware, which stops
      // the motors. Better than driving on stale feedback.
      return return_type::ERROR;
    }
  }

  for (auto & m : motors_) {
    if (m.has_hardware()) {
      read_motor(m, period.seconds());
    } else {
      m.current = 0.0;
    }

    // Joints with no hardware, or an actuator with no calibrated sensor: estimate from the
    // command, so the controllers and everything downstream keep working.
    const bool estimate = !m.has_hardware() || (m.talon_fxs && !m.measures_extension());
    if (estimate) {
      m.velocity = get_command(m.cmd_velocity);
      if (!std::isfinite(m.velocity)) {
        m.velocity = 0.0;
      }
      m.position += m.velocity * period.seconds();
      // A real joint stops at the end of its travel, so the estimate should too.
      if (m.min_position && m.position < *m.min_position) {
        m.position = *m.min_position;
        m.velocity = 0.0;
      }
      if (m.max_position && m.position > *m.max_position) {
        m.position = *m.max_position;
        m.velocity = 0.0;
      }
    }

    if (m.has_position) {set_state(m.st_position, m.position);}
    if (m.has_velocity) {set_state(m.st_velocity, m.velocity);}
    if (m.has_effort) {set_state(m.st_effort, m.current);}
  }

  publish_feedback(time);
  publish_position(time);
  return return_type::OK;
}

// =====================================================================================
// write(): turn joint velocity commands into device requests.
// =====================================================================================
return_type CanivoreHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (signals_.empty()) {
    return return_type::OK;  // nothing connected
  }

  // Safety feed: the motors only stay enabled while this keeps being called. If the controller
  // manager crashes or stalls, they disable themselves after feed_timeout_ms_.
  ctre::phoenix::unmanaged::FeedEnable(feed_timeout_ms_);

  for (auto & m : motors_) {
    if (!m.has_hardware()) {
      continue;
    }
    double command = get_command(m.cmd_velocity);
    if (!std::isfinite(command)) {
      command = 0.0;
    }

    if (m.talon_fx) {
      // Closed-loop velocity on the motor; soft limits are configured on the device.
      if (m.max_velocity > 0.0) {
        command = std::clamp(command, -m.max_velocity, m.max_velocity);
      }
      m.talon_fx->SetControl(
        m.velocity_request
        .WithVelocity(units::turns_per_second_t{rad_to_turns(command)})
        .WithEnableFOC(enable_foc_));
      continue;
    }

    // Linear actuator: open-loop output, command / max_velocity, like the old ESP32.
    if (m.extension) {
      // Stop at the ends of travel. A positive command moves the arm toward
      // feedback_angle_end; whether that extends or retracts follows from the mapping.
      const double ext_per_angle = (m.feedback_output_end - m.feedback_output_start) /
        (m.feedback_angle_end - m.feedback_angle_start);
      const double ext_direction = command * ext_per_angle;
      if ((*m.extension >= 1.0 && ext_direction > 0.0) ||
        (*m.extension <= 0.0 && ext_direction < 0.0))
      {
        command = 0.0;
      }
    }
    const double output = std::clamp(command / m.max_velocity, -1.0, 1.0);
    m.talon_fxs->SetControl(m.duty_cycle_request.WithOutput(output));
  }
  return return_type::OK;
}

// /position: the arms' 0..1 actuator extension that dig/dump and the control station use.
// Measured directly when the actuator has a calibrated sensor (like the old ESP32), otherwise
// calculated from the arm angle with the same mapping the simulation uses.
void CanivoreHardware::publish_position(const rclcpp::Time & time)
{
  if (!position_pub_ || position_publish_hz_ <= 0.0) {
    return;
  }
  const double now = time.seconds();
  if (now - last_position_publish_s_ < 1.0 / position_publish_hz_) {
    return;
  }
  const auto * front = find("front_arm_joint");
  const auto * rear = find("rear_arm_joint");
  if (!front || !rear) {
    return;  // not this robot's joint layout
  }
  last_position_publish_s_ = now;

  auto extension = [](const JointMotor & m) {
      const double ext = m.extension ? *m.extension : m.angle_to_extension(m.position);
      return static_cast<float>(std::clamp(ext, 0.0, 1.0));
    };
  serial_msgs::msg::Position msg;
  msg.front_actuator_position = extension(*front);
  msg.back_actuator_position = extension(*rear);
  position_pub_->publish(msg);
}

JointMotor * CanivoreHardware::find(const std::string & joint_name)
{
  for (auto & m : motors_) {
    if (m.name == joint_name) {
      return &m;
    }
  }
  return nullptr;
}

void CanivoreHardware::publish_feedback(const rclcpp::Time & time)
{
  if (!current_bus_pub_ || feedback_publish_hz_ <= 0.0) {
    return;
  }
  const double now = time.seconds();
  if (now - last_feedback_publish_s_ < 1.0 / feedback_publish_hz_) {
    return;
  }
  last_feedback_publish_s_ = now;

  auto current = [this](const char * joint) -> float {
      const auto * m = find(joint);
      return m ? static_cast<float>(m->current) : 0.0f;
    };
  auto temperature = [this](const char * joint) -> float {
      const auto * m = find(joint);
      return m ? static_cast<float>(m->temperature) : 0.0f;
    };

  serial_msgs::msg::CurrentBusVoltage bus;
  bus.front_left_wheel_current = current("front_left_wheel_joint");
  bus.front_right_wheel_current = current("front_right_wheel_joint");
  bus.back_left_wheel_current = current("rear_left_wheel_joint");
  bus.back_right_wheel_current = current("rear_right_wheel_joint");
  bus.front_drum_current = current("front_drum_joint");
  bus.back_drum_current = current("rear_drum_joint");
  bus.front_actuator_current = current("front_arm_joint");
  bus.back_actuator_current = current("rear_arm_joint");
  // Every TalonFX measures its own supply voltage; average them as the main battery voltage.
  // Nothing on the CAN bus measures the aux battery, so it stays 0.
  double voltage_sum = 0.0;
  int voltage_count = 0;
  for (const auto & m : motors_) {
    if (m.has_hardware()) {
      voltage_sum += m.supply_voltage;
      ++voltage_count;
    }
  }
  bus.main_battery_voltage = voltage_count ? static_cast<float>(voltage_sum / voltage_count) : 0.0f;
  bus.aux_battery_voltage = 0.0f;
  current_bus_pub_->publish(bus);

  serial_msgs::msg::Temperature temp;
  temp.front_left_wheel_temperature = temperature("front_left_wheel_joint");
  temp.front_right_wheel_temperature = temperature("front_right_wheel_joint");
  temp.back_left_wheel_temperature = temperature("rear_left_wheel_joint");
  temp.back_right_wheel_temperature = temperature("rear_right_wheel_joint");
  temp.front_drum_temperature = temperature("front_drum_joint");
  temp.back_drum_temperature = temperature("rear_drum_joint");
  temperature_pub_->publish(temp);
}

}  // namespace canivore_hardware

PLUGINLIB_EXPORT_CLASS(canivore_hardware::CanivoreHardware, hardware_interface::SystemInterface)
