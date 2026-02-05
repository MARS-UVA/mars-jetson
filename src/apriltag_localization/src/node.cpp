#include <fstream>
#include <functional>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <tf2_eigen/tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/tf2_ros/transform_broadcaster.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "field.hpp"
#include "localizer.hpp"

template <typename T>
using immutable_shared_ptr = const std::shared_ptr<const T>;

class AprilTagLocalizationNode : public rclcpp::Node {

    std::unique_ptr<apriltag::CameraLocalizer> _localizer;
    image_transport::ImageTransport _image_transport { shared_from_this() };
    image_transport::CameraSubscriber _camera_subscriber;
    tf2_ros::TransformBroadcaster _camera_pose_broadcaster { this };

public:
    AprilTagLocalizationNode() : rclcpp::Node("awareness/apriltag_localization") {
        using std::placeholders::_1, std::placeholders::_2;
        declare_parameter<std::string>("field");

        std::string field_path;
        get_parameter<std::string>("field", field_path);
        std::shared_ptr<apriltag::AprilTagField> field = apriltag::AprilTagField::parse(std::ifstream(field_path));
        _localizer = std::make_unique<apriltag::CameraLocalizer>(field, apriltag::PnPMethod::SQPNP);

        _camera_subscriber = _image_transport.subscribeCamera(
            "awareness/arducam1/image_raw",
            10,
            std::bind(&AprilTagLocalizationNode::image_callback, this, _1, _2)
        );
    }

    void image_callback(immutable_shared_ptr<sensor_msgs::msg::Image> image, immutable_shared_ptr<sensor_msgs::msg::CameraInfo> camera_info) {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::MONO8);
        cv::Mat camera_matrix { camera_info->r, false };
        std::optional<apriltag::CameraLocalizationResult> result = _localizer->localize(
            cv_ptr->image,
            camera_matrix.reshape(1, {3, 3}),
            camera_info->d
        );
        if (result.has_value()) {
            RCLCPP_INFO_STREAM(get_logger(), "Got pose " << result->estimate.pose.affine());
            geometry_msgs::msg::TransformStamped transform = tf2::eigenToTransform(result->estimate.pose);
            transform.header = image->header;
            transform.header.frame_id = "odom";  // from
            transform.child_frame_id = "arducam1_optical";  // to
            _camera_pose_broadcaster.sendTransform(transform);
        }
    }

};

int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AprilTagLocalizationNode>());
}
