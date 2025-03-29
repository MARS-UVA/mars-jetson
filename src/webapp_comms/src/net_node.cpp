#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include "std_msgs/msg/string.hpp"
#include <teleop_msgs/msg/gamepad_state.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include "main.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <nlohmann/json.hpp>

#include "./client.hpp"
#include "./server.hpp"
#include <regex>

using namespace std::chrono_literals;
using std::placeholders::_1;
using json = nlohmann::json;

const char CONTROL_STATION_IP[] = "127.0.0.1";
ThreadInfo info;

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
    publisher_ = this->create_publisher<teleop_msgs::msg::GamepadState>("gamepad_state", 10);
    timer_ = this->create_wall_timer(10ms, std::bind(&NetNode::timer_callback, this));
    motor_cmd_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("motor_cmds", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "webcam_image", 10, std::bind(&NetNode::topic_callback, this, _1));
  }

private:
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "1");
    RCLCPP_INFO(this->get_logger(), "Recieved a webcam frame");
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = cv_ptr->image;
    RCLCPP_INFO(this->get_logger(), "2");

    client_send(CONTROL_STATION_IP, img, IMAGE_PORT);
    RCLCPP_INFO(this->get_logger(), "Sent");
  }

  void timer_callback()
  {
    std::string message;
    auto motor_cmd_message = std_msgs::msg::Float32MultiArray();
    auto msg = teleop_msgs::msg::GamepadState();
    if (info.flag == true)
    {
	    //std::cout << "3.-1" << std::endl;
      RCLCPP_INFO(this->get_logger(), "3");
      message = std::string(info.client_message);
      RCLCPP_INFO(this->get_logger(), info.client_message);
      RCLCPP_INFO(this->get_logger(), "4");

      json data = json::parse(message);
      float left_stick_x = data["leftStick"]["x"];
      RCLCPP_INFO(this->get_logger(), "%f", left_stick_x);

      // This block is handling everything in the JSON individually.
      msg.left_stick.x = data["leftStick"]["x"];
      msg.left_stick.y = data["leftStick"]["y"];
      msg.right_stick.x = data["rightStick"]["x"];
      msg.right_stick.y = data["rightStick"]["y"];

      info.flag = false;
      memset(info.client_message, '\0', sizeof(info.client_message));
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", "something");

      std::regex pattern("\"([^\"]+)\"\\s*:\\s*([0-9]*\\.?[0-9]+)");

      std::sregex_iterator iter(message.begin(), message.end(), pattern);
      std::sregex_iterator end;

      while (iter != end) {
        std::smatch match = *iter;
        std::string key = match[1].str();
        double value = std::stod(match[2].str());
	RCLCPP_INFO(this->get_logger(), std::to_string(value).c_str());
        //std::cout << "Key: " << key << ", Value: " << value << std::endl;
	motor_cmd_message.data.push_back(static_cast<float>(value));
        ++iter;
      }
      motor_cmd_publisher_->publish(motor_cmd_message);
      publisher_->publish(msg);
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<teleop_msgs::msg::GamepadState>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_cmd_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
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
