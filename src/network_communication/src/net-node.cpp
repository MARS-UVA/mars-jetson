#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include "main.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

ThreadInfo info;

class NetworkNode : public rclcpp::Node {
  public:
    NetworkNode()
    : Node("Network_Node") {
      subscription_ = this->create_subscription<std_msgs::msg::String>("feedback", 10, std::bind(&NetworkNode::topic_callback, this, _1));
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&NetworkNode::timer_callback, this));
    }

  private:
    void timer_callback(){
        if (info.flag){
            auto message = std_msgs::msg::String();
            //Interpret Numbers
             uint16_t dataType = (info.client_message[1] << 8) | info.client_message[0];
             uint16_t packetNum = (info.client_message[3] << 8) | info.client_message[2];
             uint16_t totalPackets = (info.client_message[5] << 8) | info.client_message[4];
             uint16_t fragmentSize = (info.client_message[7] << 8) | info.client_message[6];
             uint16_t crc = (info.client_message[9] << 8) | info.client_message[8];
            //Extract message
            char* data = new char[fragmentSize];
            std::memcpy(&data[0], &info.client_message +10, fragmentSize);
            
            message.data = data;
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            publisher_->publish(message);
            info.flag = false;
        }
      
    }

    void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
      RCLCPP_INFO(this->get_logger(), "Received message to send");

      const std::string& str = msg->data;
      size_t bytes_to_send = 32;

      unsigned char buffer[32] = {0};
      std::memcpy(buffer, str.data(), 32);

      client_send(buffer, 32);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};


int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  info.flag = false;
  memset (info.client_message, '\0', sizeof(info.client_message));
  std::thread socket(create_server, &info);
  rclcpp::spin(std::make_shared<NetworkNode>());
  socket.join();
  rclcpp::shutdown();

  return 0;
}