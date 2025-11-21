#include <rclcpp/rclcpp.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>

class CameraDisplayer : virtual public rclcpp::Node {

    std::shared_ptr<image_transport::ImageTransport> _transport;
    image_transport::Subscriber _subscriber;
    std::shared_ptr<rclcpp::Node> _shared_this;

public:

    CameraDisplayer() : rclcpp::Node("camera_displayer") {
    }

    void setup() {
        using std::placeholders::_1;
        _transport = std::make_shared<image_transport::ImageTransport>(shared_from_this());
        _subscriber = _transport->subscribe("image_raw", 1, std::bind(&CameraDisplayer::image_callback, this, _1));
    }

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image) const {
        try {
            cv::imshow("camera", cv_bridge::toCvShare(image, "mono8")->image);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'mono8'", image->encoding.c_str());
        }
    }
};

int main(int argc, const char *const argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<CameraDisplayer> node = std::make_shared<CameraDisplayer>();
    node->setup();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}