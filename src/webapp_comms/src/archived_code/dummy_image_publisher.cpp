//Standard Libraries
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdio.h>
#include <iostream>

//ROS C++ libraries:
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

//OpenCV and OpenCV-to-ROS message CVBridge library includes:
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/core/types.hpp>
#include <opencv2/core/hal/interface.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc/imgproc.hpp>
 
using namespace std::chrono_literals;
using namespace cv;
 
class WebcamVideoPublisher : public rclcpp::Node {  //WebcamVideoPublisher is a child class of the ROS Node class
public:
  WebcamVideoPublisher() : Node("webcam_publisher"), count_(0) //Class constructor with node name passed to super class
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("webcam_topic", 10);  //Publisher object created
    timer_ = this->create_wall_timer(500ms, std::bind(&WebcamVideoPublisher::timer_callback, this));  //Timer object created, ROS uses timers frequently in publishers sort of as a set time or polling window to perform an action. As you can see, the timer is triggered very frequently, every 500 ms
  }
 
private:
    void timer_callback()  //timer callback function called every 500ms
    {
      cv_bridge::CvImagePtr cv_ptr;  //New cv_bridge object instantiated
      cv::Mat img(800, 800, CV_8UC3); //New image matrix object instantiated with 800x800 pixels size, CV_8UC3 is just a fancy name for RGB values or 3 channels, each enncoded with 8 bits from 0-255
      cv::randu(img, Scalar(0, 0, 0), Scalar(255, 255, 255));  //Assigning a random value to each element (or pixel) in the matrix from 0-255

      sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();  ///Creating a ROS message of type Image and using CVBridge to convert our image matrix from before to an Image message type
      // BTW, SharedPtr or shared pointers are a crazy cool concept in C (C16 i think), Rust, and C++ which allows multiple instances to share ownership of an object and the object is destroyed when all instances pointing to the object are destroyed
      // Useful in ROS because we are creating a nnnew frame every 500 ms which can be efficently utilized in memory with SharedPtr

      auto message = std_msgs::msg::String(); //Just a message that is created to put in the ROS logger
      message.data = "Publishing image frame " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

      publisher_->publish(*msg.get());  //Publish our image message in the variable 'msg'
      std::cout << "Image frame published!" << std::endl;
    }
    // Creating a private instance of the timer_ and publisher_ which we officially define publicly in the constructor
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    size_t count_;
};

// First point of contact is the mmain() ffunction which will initialize the node (by calling its constructor) and "spinning" it (starting it)
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WebcamVideoPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
