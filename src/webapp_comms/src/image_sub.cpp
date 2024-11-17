#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

extern "C" {
    #include "client_udp.c"  // Ensure the C code is available to be compiled
}

class WebcamSubscriber : public rclcpp::Node {
public:
    WebcamSubscriber() : Node("webcam_subscriber") {
        RCLCPP_INFO(this->get_logger(), "Webcam subscriber node initialized");

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "webcam_topic", 
            10, 
            std::bind(&WebcamSubscriber::listener_callback, this, std::placeholders::_1)
        );
    }

private:
void listener_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
// Convert ROS image message to OpenCV cv::Mat
    cv::Mat image;
    try {
        // Use cv_bridge to convert the ROS image message into a cv::Mat
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        image = cv_ptr->image;  // Get the OpenCV image
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Encode the cv::Mat to a byte array (compressed image)
    std::vector<uchar> buffer;
    bool success = cv::imencode(".jpg", image, buffer);  // JPEG compression
    if (!success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to encode image.");
        return;
    }

    // get pointer for char* to send over sockeet
    unsigned char* char_ptr = reinterpret_cast<unsigned char*>(buffer.data());
    


    client("127.0.0.1", char_ptr);

    std::cout << "Raw data: ";
    for (unsigned char c : char_ptr) {
        std::cout << c; // Prints as raw characters (may not be readable)
        RCLCPP_INFO(this->get_logger(), static_cast<const char>(c));
    }
    std::cout << std::endl;

    

}

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WebcamSubscriber>();
    rclcpp::spin(node);
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
