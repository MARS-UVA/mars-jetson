#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "std_msgs/msg/string.hpp"
#include <opencv2/opencv.hpp>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "./client.hpp"
#include "./server.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

const char CONTROL_STATION_IP[] = "127.0.0.1";

const int MOTOR_CURRENT_BYTES = 4;
int Socket (ThreadInfo* info) {
  create_server(info);
  return 0;
}

class NetNode : public rclcpp::Node
{
  public:
    NetNode()
    : Node("NetNode"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(100000ms, std::bind(&NetNode::timer_callback, this));
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "webcam_image", 10, std::bind(&NetNode::topic_callback, this, _1));
    }

  private:
    ThreadInfo info;
    void topic_callback (const sensor_msgs::msg::Image::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "Recieved a webcam frame");
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv::Mat img = cv_ptr->image;

      client_send(CONTROL_STATION_IP, img);
      RCLCPP_INFO(this->get_logger(), "Sent");
    }

    void timer_callback()
    {
      RCLCPP_INFO(this->get_logger(), "Started timer_callback");
      auto message = std_msgs::msg::String();
      while(info.flag==false){}
      if(info.flag == true){
        message.data = info.client_message;
        info.flag = false;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
      }
      RCLCPP_INFO(this->get_logger(), "Ending timer_callback");
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  ThreadInfo info;
  info.flag = false;
  info.client_message;
  std::thread socket (Socket, &info);
  rclcpp::spin(std::make_shared<NetNode>());
  socket.join();
  rclcpp::shutdown();
  return 0;
}