#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "./client.hpp"
#include "./server.hpp"

using namespace std::chrono_literals;

const String CONTROL_STATION_IP = "127.0.0.1"

const int MOTOR_CURRENT_BYTES = 4;
void Socket (ThreadInfo* info) {
  create_server(info)
}

class NetNode : public rclcpp::Node
{
  public:
    NetNode()
    : Node("NetNode"), count_(0)
    {
      threadInfo info;
      info.flag = false;
      info.client_message = "";
      std::thread socket (Socket, info)
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      10ms, std::bind(&MinimalPublisher::timer_callback, this));
      subscription_ = this->create_subscriber<std_msgs:msg::String>(
      "webcam_image", 10, std::bind(&netNode::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Recieved a webcam frame");
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)
      cv::Mat img = cv_ptr->img

      client_send(CONTROL_STATION_IP, img)
    }

    void timer_callback()
    auto message = std_msgs::msg::String();
    {
      while(info.flag==false){}
      if(info.flag == true){
        message.data = info.client_message;
      }
      info.flag = false
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NetNode>());
  socket.join();
  rclcpp::shutdown();
  return 0;
}