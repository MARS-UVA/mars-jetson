#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class SimplePublisher : public rclcpp::Node
{
public:
    SimplePublisher()
    : Node("simple_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("feedback", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&SimplePublisher::timer_callback, this));

        // Precompute 64-character string
        message_.data = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        if (message_.data.length() != 64) {
            RCLCPP_WARN(this->get_logger(), "String length is not 64 characters!");
        }
    }

private:
    void timer_callback()
    {
        publisher_->publish(message_);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_.data.c_str());
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std_msgs::msg::String message_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplePublisher>());
    rclcpp::shutdown();
    return 0;
}
