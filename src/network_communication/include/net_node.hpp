#include <chrono>
#include <functional>
#include <memory>
#include <std_msgs/msg/detail/u_int8__struct.hpp>
#include <string>
#ifdef LEGACY_CV_BRIDGE
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif

#include <cmath>
#include <regex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <sensor_msgs/msg/image.hpp>

#include <teleop_msgs/msg/gamepad_state.hpp>
#include <teleop_msgs/msg/stick_position.hpp>
#include "teleop_msgs/msg/human_input_state.hpp"
#include <serial_msgs/msg/current_bus_voltage.hpp>
#include <serial_msgs/msg/position.hpp>
#include <serial_msgs/msg/temperature.hpp>
#include <autonomy_msgs/action/autonomous_actions.hpp>

#include "main.hpp"
#include "client.hpp"
#include "server.hpp"


#define NUM_GAMEPAD_BTNS 14
#define NUM_GAMEPAGE_STICKS 2
#define DIG_AUTO 1
#define DUMP_AUTO 2
#define ESTOP 3
#define TRAVERSAL_AUTO 4

using namespace std::chrono_literals;
using std::placeholders::_1;
using teleop_msgs::msg::GamepadState;
using teleop_msgs::msg::StickPosition;
using serial_msgs::msg::CurrentBusVoltage;
using serial_msgs::msg::Position;
using serial_msgs::msg::Temperature;

using FieldPtr = bool teleop_msgs::msg::GamepadState::*;

const std::vector<std::pair<std::string, FieldPtr>> fields = {
    {"x_pressed", (FieldPtr)&teleop_msgs::msg::GamepadState::x_pressed},
    {"y_pressed", (FieldPtr)&teleop_msgs::msg::GamepadState::y_pressed},
    {"a_pressed", (FieldPtr)&teleop_msgs::msg::GamepadState::a_pressed},
    {"b_pressed", (FieldPtr)&teleop_msgs::msg::GamepadState::b_pressed},
    {"lt_pressed", (FieldPtr)&teleop_msgs::msg::GamepadState::lt_pressed},
    {"rt_pressed", (FieldPtr)&teleop_msgs::msg::GamepadState::rt_pressed},
    {"lb_pressed", (FieldPtr)&teleop_msgs::msg::GamepadState::lb_pressed},
    {"rb_pressed", (FieldPtr)&teleop_msgs::msg::GamepadState::rb_pressed},
    {"dd_pressed", (FieldPtr)&teleop_msgs::msg::GamepadState::dd_pressed},
    {"du_pressed", (FieldPtr)&teleop_msgs::msg::GamepadState::du_pressed},
    {"l3_pressed", (FieldPtr)&teleop_msgs::msg::GamepadState::l3_pressed},
    {"r3_pressed", (FieldPtr)&teleop_msgs::msg::GamepadState::r3_pressed},
    {"back_pressed", (FieldPtr)&teleop_msgs::msg::GamepadState::back_pressed},
    {"start_pressed", (FieldPtr)&teleop_msgs::msg::GamepadState::start_pressed}};

const size_t buffer_size = 72;

class NetNode : public rclcpp::Node
{
public:
  using DigDump = autonomy_msgs::action::AutonomousActions;
  using DigDumpGoalHandle = rclcpp_action::ServerGoalHandle<DigDump>;
  unsigned char* buffer;
  NetNode();

  void digdump_send_goal(int action_type);
  void digdump_cancel_goal();

private:
  void current_bus_voltage_callback(const serial_msgs::msg::CurrentBusVoltage::SharedPtr msg);
  void temperature_callback(const serial_msgs::msg::Temperature::SharedPtr msg);
  void position_callback(const serial_msgs::msg::Position::SharedPtr msg);
  
  // TODO: send to UI
  void robot_state_callback(const std_msgs::msg::UInt8::SharedPtr state);
  void digdump_goal_response_callback(const rclcpp_action::ClientGoalHandle<DigDump>::SharedPtr & goal_handle);

  void serial_timer_callback();
  void action_timer_callback();
  void controller_timer_callback();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr serialTimer_;
  rclcpp::TimerBase::SharedPtr timertwo_;
  rclcpp::Publisher<teleop_msgs::msg::HumanInputState>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr robot_state_toggle_publisher_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr robot_state_subscriber_;
  rclcpp::Subscription<serial_msgs::msg::CurrentBusVoltage>::SharedPtr currentBusVoltageSubscription_;
  rclcpp::Subscription<serial_msgs::msg::Temperature>::SharedPtr temperatureSubscription_;
  rclcpp::Subscription<serial_msgs::msg::Position>::SharedPtr positionSubscription_;

  rclcpp_action::Client<DigDump>::SharedPtr digdump_client_ptr_;
  rclcpp_action::ClientGoalHandle<autonomy_msgs::action::AutonomousActions>::SharedPtr digdump_goal_handle;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_purepursuit_client_;
  /// Avoid spamming start_purepursuit on every controller timer tick while in TRAVERSAL_AUTO.
  bool pure_pursuit_start_sent_{false};
};