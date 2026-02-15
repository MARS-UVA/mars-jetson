#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include "std_msgs/msg/string.hpp"
#include <teleop_msgs/msg/gamepad_state.hpp>
#include <teleop_msgs/msg/stick_position.hpp>
#include "teleop_msgs/msg/human_input_state.hpp"
#include <serial_msgs/msg/feedback.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include "main.hpp"
#include <cmath>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "client.hpp"
#include "server.hpp"
#include <regex>

#define NUM_GAMEPAD_BTNS 14
#define NUM_GAMEPAGE_STICKS 2

using namespace std::chrono_literals;
using std::placeholders::_1;
using teleop_msgs::msg::GamepadState;
using teleop_msgs::msg::StickPosition;
using serial_msgs::msg::Feedback;

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
std::map<std::string, float> complete_feedback_data = {
    {"front_left_wheel_current", 0.0f},
    {"back_left_wheel_current", 0.0f},
    {"front_right_wheel_current", 0.0f},
    {"back_right_wheel_current", 0.0f},
    {"front_drum_current", 0.0f},
    {"back_drum_current", 0.0f},
    {"front_actuator_current", 0.0f},
    {"back_actuator_current", 0.0f},
    {"main_battery_voltage", 0.0f},
    {"aux_battery_voltage", 0.0f},
    {"front_left_wheel_temperature", 0.0f},
    {"back_left_wheel_temperature", 0.0f},
    {"front_right_wheel_temperature", 0.0f},
    {"back_right_wheel_temperature", 0.0f},
    {"front_drum_temperature", 0.0f},
    {"back_drum_temperature", 0.0f},
    {"front_actuator_position", 0.0f},
    {"back_actuator_position", 0.0f}
  };

using StickFieldPtr = teleop_msgs::msg::StickPosition teleop_msgs::msg::GamepadState::*;
std::vector<std::pair<std::string, StickFieldPtr>> stickFields = {
    {"left_stick", &teleop_msgs::msg::GamepadState::left_stick},
    {"right_stick", &teleop_msgs::msg::GamepadState::right_stick}};

// ImageReader imgReader;

//const char* CONTROL_STATION_IP_CLIENT = std::getenv("CONTROL_STATION_IP");
ThreadInfo info;
int counter = 0;

const int MOTOR_CURRENT_BYTES = 4;
int Socket(ThreadInfo *info)
{
  create_server(info);
  return 0;
}

class NetNode : public rclcpp::Node
{
public:
  NetNode()
      : Node("NetNode"), count_(0)
  {
    publisher_ = this->create_publisher<teleop_msgs::msg::HumanInputState>("human_input_state", 10);
    timer_ = this->create_wall_timer(10ms, std::bind(&NetNode::timer_callback, this));
    subscription_ = this->create_subscription<serial_msgs::msg::Feedback>(
        "feedback", 10, std::bind(&NetNode::topic_callback, this, _1));
    currentBusVoltageSubscription_ = this->create_subscription<serial_msgs::msg::CurrentBusVoltage>(
        "bus_voltage", 10, std::bind(&NetNode::current_bus_voltage_callback, this, _1)
    )
    temperatureSubscription_ = this->create_subscription<serial_msgs::msg::Temperature>(
        "temperature", 10, std::bind(&NetNode::temperature_callback, this, _1)
    );
    positionSubscription_ = this->create_subscription<serial_msgs::msg::Position>(
        "position", 10, std::bind(&NetNode::position_callback, this, _1)
    );
  }

private:
  void topic_callback(const serial_msgs::msg::Feedback::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received motor feedback packet");
    float front_left = msg->front_left;
    float front_right = msg->front_right;
    float back_left = msg->back_left;
    float back_right = msg->back_right;
    float l_bucket_drum = msg->l_drum;
    float r_bucket_drum = msg->r_drum;
    float l_actuator = msg->l_actuator;
    float r_actuator = msg->r_actuator;
    float actuator_height = msg->actuator_height;

    size_t buffer_size = 36;
    unsigned char* buffer = new unsigned char[buffer_size];
    std::memcpy(&buffer[0], &front_left, 4);
    std::memcpy(&buffer[4], &front_right, 4);
    std::memcpy(&buffer[8], &back_left, 4);
    std::memcpy(&buffer[12], &back_right, 4);
    std::memcpy(&buffer[16], &l_bucket_drum, 4);
    std::memcpy(&buffer[20], &r_bucket_drum, 4);
    std::memcpy(&buffer[24], &l_actuator, 4);
    std::memcpy(&buffer[28], &r_actuator, 4);
    std::memcpy(&buffer[32], &actuator_height, 4);

    // RCLCPP_WARN(this->get_logger(), "Sending now...");
    client_send(buffer, buffer_size, CURRENT_FEEDBACK_PORT);
  }

  void send_updated_data() {
    size_t buffer_size = 72;
    unsigned char* buffer = new unsigned char[buffer_size];
    std::memcpy(&buffer[0], &complete_feedback_data["front_left_wheel_current"], 4);
    std::memcpy(&buffer[4], &complete_feedback_data["back_left_wheel_current"], 4);
    std::memcpy(&buffer[8], &complete_feedback_data["front_right_wheel_current"], 4);
    std::memcpy(&buffer[12], &complete_feedback_data["back_right_wheel_current"], 4);
    std::memcpy(&buffer[16], &complete_feedback_data["front_drum_current"], 4);
    std::memcpy(&buffer[20], &complete_feedback_data["back_drum_current"], 4);
    std::memcpy(&buffer[24], &complete_feedback_data["front_actuator_current"], 4);
    std::memcpy(&buffer[28], &complete_feedback_data["back_actuator_current"], 4);
    std::memcpy(&buffer[32], &complete_feedback_data["main_battery_voltage"], 4);
    std::memcpy(&buffer[36], &complete_feedback_data["aux_battery_voltage"], 4);
    std::memcpy(&buffer[40], &complete_feedback_data["front_left_wheel_temperature"], 4);
    std::memcpy(&buffer[44], &complete_feedback_data["back_left_wheel_temperature"], 4);
    std::memcpy(&buffer[48], &complete_feedback_data["front_right_wheel_temperature"], 4);
    std::memcpy(&buffer[52], &complete_feedback_data["back_right_wheel_temperature"], 4);
    std::memcpy(&buffer[56], &complete_feedback_data["front_drum_temperature"], 4);
    std::memcpy(&buffer[60], &complete_feedback_data["back_drum_temperature"], 4);
    std::memcpy(&buffer[64], &complete_feedback_data["front_actuator_position"], 4);
    std::memcpy(&buffer[68], &complete_feedback_data["back_actuator_position"], 4);

    client_send(buffer, buffer_size, CURRENT_FEEDBACK_PORT);
  }

  void current_bus_voltage_callback(const serial_msgs::msg::CurrentBusVoltage::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received current bus voltage feedback packet");

    complete_feedback_data["front_left_wheel_current"] = msg->front_left_wheel_current;
    complete_feedback_data["back_left_wheel_current"] = msg->back_left_wheel_current;
    complete_feedback_data["front_right_wheel_current"] = msg->front_right_wheel_current;
    complete_feedback_data["back_right_wheel_current"] = msg->back_right_wheel_current;
    complete_feedback_data["front_drum_current"] = msg->front_drum_current;
    complete_feedback_data["back_drum_current"] = msg->back_drum_current;
    complete_feedback_data["front_actuator_current"] = msg->front_actuator_current;
    complete_feedback_data["back_actuator_current"] = msg->back_actuator_current;
    complete_feedback_data["main_battery_voltage"] = msg->main_battery_voltage;
    complete_feedback_data["aux_battery_voltage"] = msg->aux_battery_voltage;

    send_updated_data();
  }



  void temperature_callback(const serial_msgs::msg::Temperature::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received temperature feedback packet");

    complete_feedback_data["front_left_wheel_temperature"] = msg->front_left_wheel_temperature;
    complete_feedback_data["back_left_wheel_temperature"] = msg->back_left_wheel_temperature;
    complete_feedback_data["front_right_wheel_temperature"] = msg->front_right_wheel_temperature;
    complete_feedback_data["back_right_wheel_temperature"] = msg->back_right_wheel_temperature;
    complete_feedback_data["front_drum_temperature"] = msg->front_drum_temperature;
    complete_feedback_data["back_drum_temperature"] = msg->back_drum_temperature;

    send_updated_data();
  }

  void position_callback(const serial_msgs::msg::Position::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received position feedback packet");

    complete_feedback_data["front_actuator_position"] = msg->front_actuator_position;
    complete_feedback_data["back_actuator_position"] = msg->back_actuator_position;

    send_updated_data();
  }

  void timer_callback()
  {
    /* Receive Control messages */
    counter++;
    std::string message;
    auto stickPosition_msg = std::make_shared<StickPosition>();
    auto gamepad_msg = std::make_shared<GamepadState>();
    auto human_input_msg = std::make_shared<teleop_msgs::msg::HumanInputState>();
    if (info.flag == true)
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

      info.flag = false;
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

      human_input_msg->gamepad_state = *gamepad_msg;
      human_input_msg->drive_mode = human_input_msg->DRIVEMODE_TELEOP;
      human_input_msg->a_stop = false;
      human_input_msg->e_stop = false;

      RCLCPP_INFO(this->get_logger(), "Publishing HumanInputState: drive_mode = %d", human_input_msg->drive_mode);

      publisher_->publish(*human_input_msg);
    }

    /* Send Webcam Feeds over */
    // cv::Mat fromCam = imgReader.processImage();
    // if (!fromCam.empty())
    // {
    //   client_send(CONTROL_STATION_IP, fromCam, IMAGE_PORT);
    //   RCLCPP_INFO(this->get_logger(), "Sent webcam feed");
    // }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<teleop_msgs::msg::HumanInputState>::SharedPtr publisher_;
  rclcpp::Subscription<serial_msgs::msg::Feedback>::SharedPtr subscription_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  info.flag = false;
  memset(info.client_message, '\0', sizeof(info.client_message));
  std::thread socket(create_server, &info);
  rclcpp::spin(std::make_shared<NetNode>());
  socket.join();
  rclcpp::shutdown();
  return 0;
}
