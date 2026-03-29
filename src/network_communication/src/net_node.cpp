#include <chrono>
#include <functional>
#include <memory>
#include <std_msgs/msg/detail/u_int8__struct.hpp>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include <teleop_msgs/msg/gamepad_state.hpp>
#include <teleop_msgs/msg/stick_position.hpp>
#include "teleop_msgs/msg/human_input_state.hpp"
#include <serial_msgs/msg/current_bus_voltage.hpp>
#include <serial_msgs/msg/position.hpp>
#include <serial_msgs/msg/temperature.hpp>
#include <autonomy_msgs/action/autonomous_actions.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include "main.hpp"
#include <cmath>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "client.hpp"
#include "server.hpp"
#include <regex>

#define NUM_GAMEPAD_BTNS 14
#define NUM_GAMEPAGE_STICKS 2
#define DIG_AUTO 1
#define DUMP_AUTO 2
#define ESTOP 3

using namespace std::chrono_literals;
using std::placeholders::_1;
using teleop_msgs::msg::GamepadState;
using teleop_msgs::msg::StickPosition;
using serial_msgs::msg::CurrentBusVoltage;
using serial_msgs::msg::Position;
using serial_msgs::msg::Temperature;

using FieldPtr = bool teleop_msgs::msg::GamepadState::*;
std::vector<std::pair<std::string, FieldPtr>> fields = {
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

size_t buffer_size = 72;
unsigned char* buffer = new unsigned char[buffer_size];


using StickFieldPtr = teleop_msgs::msg::StickPosition teleop_msgs::msg::GamepadState::*;
std::vector<std::pair<std::string, StickFieldPtr>> stickFields = {
    {"left_stick", &teleop_msgs::msg::GamepadState::left_stick},
    {"right_stick", &teleop_msgs::msg::GamepadState::right_stick}};

// ImageReader imgReader;

//const char* CONTROL_STATION_IP_CLIENT = std::getenv("CONTROL_STATION_IP");
ThreadInfo info;
int counter = 0;
uint8_t current_action_state = 0;

const int MOTOR_CURRENT_BYTES = 4;
int Socket(ThreadInfo *info)
{
  create_server(info);
  return 0;
}

class NetNode : public rclcpp::Node
{
public:
  using DigDump = autonomy_msgs::action::AutonomousActions;
  using DigDumpGoalHandle = rclcpp_action::ServerGoalHandle<DigDump>;
  NetNode()
      : Node("NetNode")
  {
    goal_handle = nullptr;
    client_ptr_ = rclcpp_action::create_client<DigDump>(this, "digdump");
    robot_state_toggle_publisher_ = this->create_publisher<std_msgs::msg::UInt8>("robot_state/toggle", 10);
    robot_state_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>("robot_state", 10, std::bind(&NetNode::robot_state_callback, this, _1));
    publisher_ = this->create_publisher<teleop_msgs::msg::HumanInputState>("human_input_state", 10);
    timer_ = this->create_wall_timer(10ms, std::bind(&NetNode::controller_timer_callback, this));
    timertwo_ = this->create_wall_timer(10ms, std::bind(&NetNode::action_timer_callback, this));
    // subscription_ = this->create_subscription<serial_msgs::msg::Feedback>(
    //     "feedback", 10, std::bind(&NetNode::topic_callback, this, _1));
    serialTimer_ = this->create_wall_timer(10ms, std::bind(&NetNode::serial_timer_callback, this));
    currentBusVoltageSubscription_ = this->create_subscription<serial_msgs::msg::CurrentBusVoltage>(
        "current_bus_voltage", 10, std::bind(&NetNode::current_bus_voltage_callback, this, _1)
    );
    temperatureSubscription_ = this->create_subscription<serial_msgs::msg::Temperature>(
        "temperature", 10, std::bind(&NetNode::temperature_callback, this, _1)
    );
    positionSubscription_ = this->create_subscription<serial_msgs::msg::Position>(
        "position", 10, std::bind(&NetNode::position_callback, this, _1)
    );
  }

  void send_goal(int action_type) {
    
    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }

    auto goal_msg = DigDump::Goal();
    goal_msg.index = action_type;

    auto send_goal_options = rclcpp_action::Client<DigDump>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&NetNode::goal_response_callback, this, _1);

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  void cancel_goal() {
    if (this->goal_handle) {
      this->client_ptr_->async_cancel_goal(this->goal_handle);
    }
  }

private:

  void goal_response_callback(const rclcpp_action::ClientGoalHandle<DigDump>::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void current_bus_voltage_callback(const serial_msgs::msg::CurrentBusVoltage::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received current bus voltage feedback packet");

    std::memcpy(&buffer[FeedbackByteIndices::FRONT_LEFT_WHEEL_CURRENT], &msg->front_left_wheel_current, 4);
    std::memcpy(&buffer[FeedbackByteIndices::BACK_LEFT_WHEEL_CURRENT], &msg->back_left_wheel_current, 4);
    std::memcpy(&buffer[FeedbackByteIndices::FRONT_RIGHT_WHEEL_CURRENT], &msg->front_right_wheel_current, 4);
    std::memcpy(&buffer[FeedbackByteIndices::BACK_RIGHT_WHEEL_CURRENT], &msg->back_right_wheel_current, 4);
    std::memcpy(&buffer[FeedbackByteIndices::FRONT_DRUM_CURRENT], &msg->front_drum_current, 4);
    std::memcpy(&buffer[FeedbackByteIndices::BACK_DRUM_CURRENT], &msg->back_drum_current, 4);
    std::memcpy(&buffer[FeedbackByteIndices::FRONT_ACTUATOR_CURRENT], &msg->front_actuator_current, 4);
    std::memcpy(&buffer[FeedbackByteIndices::BACK_ACTUATOR_CURRENT], &msg->back_actuator_current, 4);
    std::memcpy(&buffer[FeedbackByteIndices::MAIN_BATTERY_VOLTAGE], &msg->main_battery_voltage, 4);
    std::memcpy(&buffer[FeedbackByteIndices::AUX_BATTERY_VOLTAGE], &msg->aux_battery_voltage, 4);
  }

  void temperature_callback(const serial_msgs::msg::Temperature::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received temperature feedback packet");

    std::memcpy(&buffer[FeedbackByteIndices::FRONT_LEFT_WHEEL_TEMPERATURE], &msg->front_left_wheel_temperature, 4);
    std::memcpy(&buffer[FeedbackByteIndices::BACK_LEFT_WHEEL_TEMPERATURE], &msg->back_left_wheel_temperature, 4);
    std::memcpy(&buffer[FeedbackByteIndices::FRONT_RIGHT_WHEEL_TEMPERATURE], &msg->front_right_wheel_temperature, 4);
    std::memcpy(&buffer[FeedbackByteIndices::BACK_RIGHT_WHEEL_TEMPERATURE], &msg->back_right_wheel_temperature, 4);
    std::memcpy(&buffer[FeedbackByteIndices::FRONT_DRUM_TEMPERATURE], &msg->front_drum_temperature, 4);
    std::memcpy(&buffer[FeedbackByteIndices::BACK_DRUM_TEMPERATURE], &msg->back_drum_temperature, 4);
  }

  void position_callback(const serial_msgs::msg::Position::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received position feedback packet");
    std::memcpy(&buffer[FeedbackByteIndices::FRONT_ACTUATOR_POSITION], &msg->front_actuator_position, 4);
    std::memcpy(&buffer[FeedbackByteIndices::BACK_ACTUATOR_POSITION], &msg->back_actuator_position, 4);
  }

  void robot_state_callback(const std_msgs::msg::UInt8::SharedPtr state) {
    // TODO: send to UI
  }

  void serial_timer_callback()
  {
      client_send(buffer, buffer_size, CURRENT_FEEDBACK_PORT);
  }

  void action_timer_callback() {
    uint8_t robot_action;
    if (info.auto_flag) {
      robot_action = info.robot_action;
      current_action_state = info.robot_action;
      cancel_goal();
      switch (robot_action) {
        case 0: // default, do nothing
          break;
        case ESTOP: // Estop, do nothing else, handled in controller_timer_callback
          break;
        case DIG_AUTO:
          send_goal(DIG_AUTO);
          break;
        case DUMP_AUTO:
          send_goal(DUMP_AUTO);
          break;
      }
      info.auto_flag = false;
    }
  }

  void controller_timer_callback()
  {
    /* Receive Control messages */
    counter++;
    std::string message;
    auto stickPosition_msg = std::make_shared<StickPosition>();
    auto gamepad_msg = std::make_shared<GamepadState>();
    auto human_input_msg = std::make_shared<teleop_msgs::msg::HumanInputState>();
    if (info.controller_flag == true)
    {
      message = std::string(info.client_message);

      const char delimiter[] = ",";
      char *token;

      token = strtok(info.client_message, delimiter);
      size_t field_i = 0;
      size_t subField = 0;

      while (token != NULL)
      {
        double value = 0.0;
        RCLCPP_INFO(this->get_logger(), "Token:%s", token);
        try
        {
          value = std::stod(token);
        }
        catch (...)
        {
          RCLCPP_INFO(this->get_logger(), "Invalid packet encountered");
          return;
        }
        if (field_i < NUM_GAMEPAD_BTNS)
        {
          auto field = fields[field_i];
          (gamepad_msg.get())->*(field.second) = static_cast<bool>(value);
        }
        else
        {
          auto field = stickFields[field_i - NUM_GAMEPAD_BTNS];
          if (subField == 0)
          {
            subField++;
            ((*gamepad_msg).*(field.second)).x = value;
            field_i--;
          }
          else if (subField == 1)
          {
            subField = 0;
            ((*gamepad_msg).*(field.second)).y = value;
          }
        }
        token = strtok(NULL, delimiter);
        field_i++;
      }

      info.controller_flag = false;
      memset(info.client_message, '\0', sizeof(info.client_message));

      RCLCPP_INFO(
          this->get_logger(),
          "GamepadState:\n"
          "  Buttons: x=%s, y=%s, a=%s, b=%s\n"
          "  Triggers: lt=%s, rt=%s, lb=%s, rb=%s\n"
          "  D-pad: up=%s, down=%s, left=%s, right=%s\n"
          "  Sticks: l3=%s, r3=%s\n"
          "  Menu: back=%s, start=%s\n"
          "  Left stick: (%.2f, %.2f)\n"
          "  Right stick: (%.2f, %.2f)",
          gamepad_msg->x_pressed ? "true" : "false",
          gamepad_msg->y_pressed ? "true" : "false",
          gamepad_msg->a_pressed ? "true" : "false",
          gamepad_msg->b_pressed ? "true" : "false",
          gamepad_msg->lt_pressed ? "true" : "false",
          gamepad_msg->rt_pressed ? "true" : "false",
          gamepad_msg->lb_pressed ? "true" : "false",
          gamepad_msg->rb_pressed ? "true" : "false",
          gamepad_msg->du_pressed ? "true" : "false",
          gamepad_msg->dd_pressed ? "true" : "false",
          gamepad_msg->dl_pressed ? "true" : "false",
          gamepad_msg->dr_pressed ? "true" : "false",
          gamepad_msg->l3_pressed ? "true" : "false",
          gamepad_msg->r3_pressed ? "true" : "false",
          gamepad_msg->back_pressed ? "true" : "false",
          gamepad_msg->start_pressed ? "true" : "false",
          gamepad_msg->left_stick.x, gamepad_msg->left_stick.y,
          gamepad_msg->right_stick.x, gamepad_msg->right_stick.y);
      
      switch (current_action_state) {
        case 0: //default, do nothing
          human_input_msg->drive_mode = human_input_msg->DRIVEMODE_TELEOP;
          break;
        case DIG_AUTO: //TODO: make client and send goal to action server
          human_input_msg->drive_mode = human_input_msg->DRIVEMODE_AUTONOMOUS;
          send_goal(DIG_AUTO);
          break;
        case DUMP_AUTO:
          human_input_msg->drive_mode = human_input_msg->DRIVEMODE_AUTONOMOUS;
          send_goal(DUMP_AUTO);
          break;
        case ESTOP:
          std_msgs::msg::UInt8 msg;
          msg.data = ESTOP;
          robot_state_toggle_publisher_->publish(msg); //everything else is handled
      }
      human_input_msg->gamepad_state = *gamepad_msg;

      //UNUSED 
      human_input_msg->a_stop = false;
      human_input_msg->e_stop = false;

      RCLCPP_INFO(this->get_logger(), "Publishing HumanInputState: drive_mode = %d", human_input_msg->drive_mode);

      publisher_->publish(*human_input_msg);
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr serialTimer_;
  rclcpp::TimerBase::SharedPtr timertwo_;
  rclcpp::Publisher<teleop_msgs::msg::HumanInputState>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr robot_state_toggle_publisher_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr robot_state_subscriber_;
  rclcpp::Subscription<serial_msgs::msg::CurrentBusVoltage>::SharedPtr currentBusVoltageSubscription_;
  rclcpp::Subscription<serial_msgs::msg::Temperature>::SharedPtr temperatureSubscription_;
  rclcpp::Subscription<serial_msgs::msg::Position>::SharedPtr positionSubscription_;

  rclcpp_action::Client<DigDump>::SharedPtr client_ptr_;
  rclcpp_action::ClientGoalHandle<autonomy_msgs::action::AutonomousActions>::SharedPtr goal_handle;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  info.controller_flag = false;
  info.auto_flag = false;
  memset(info.client_message, '\0', sizeof(info.client_message));
  std::thread socket(create_server, &info);
  rclcpp::spin(std::make_shared<NetNode>());
  socket.join();
  rclcpp::shutdown();
  return 0;
}
