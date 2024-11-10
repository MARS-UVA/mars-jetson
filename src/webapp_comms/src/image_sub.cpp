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
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        size_t img_size = img.total() * img.elemSize();
        std::vector<uchar> img_data(img_size);
        memcpy(img_data.data(), img.data, img_size);
        cv::imshow("Image", cv_ptr->image);
        cv::waitKey(1);
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
