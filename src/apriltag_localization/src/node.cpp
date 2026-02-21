#include <fstream>
#include <functional>
#include <memory>
#include <optional>
#include <tagStandard41h12.h>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#ifdef LEGACY_CV_BRIDGE
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif
#include <image_transport/image_transport.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2/transform_datatypes.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Geometry>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Matrix.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "field.hpp"
#include "localizer.hpp"

template <typename T>
using immutable_shared_ptr = const std::shared_ptr<const T>;

class AprilTagLocalizationNode : public rclcpp::Node {

    std::unique_ptr<apriltag::CameraLocalizer> _localizer;
    image_transport::CameraSubscriber _camera_subscriber;
    std::unique_ptr<tf2_ros::Buffer> _tf2_buffer;
    std::shared_ptr<tf2_ros::TransformListener> _tf2_listener;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _camera_pose_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _pose_publisher;

public:
    AprilTagLocalizationNode() : rclcpp::Node("apriltag_localization") {
        using std::placeholders::_1, std::placeholders::_2;
        declare_parameter<std::string>("field");

        std::string field_path;
        get_parameter<std::string>("field", field_path);
        std::shared_ptr<apriltag::AprilTagField> field = apriltag::AprilTagField::parse(std::ifstream(field_path));
        _localizer = std::make_unique<apriltag::CameraLocalizer>(field, apriltag::PnPMethod::SQPNP);

        const auto family = apriltag::AprilTagFamily::get(tagStandard41h12_create(), tagStandard41h12_destroy);
        _localizer->detector().nthreads() = 8;
        _localizer->detector().add_family(family);

        _camera_subscriber = image_transport::create_camera_subscription(
            this,
            "image_raw",
            std::bind(&AprilTagLocalizationNode::image_callback, this, _1, _2),
            "raw",
            rclcpp::SensorDataQoS().get_rmw_qos_profile()
        );
        _tf2_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        _tf2_listener = std::make_shared<tf2_ros::TransformListener>(*_tf2_buffer);
        _camera_pose_publisher = create_publisher<geometry_msgs::msg::PoseStamped>("camera_pose_estimate", rclcpp::SensorDataQoS{});
        _pose_publisher = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("robot_pose_estimate", rclcpp::SensorDataQoS{});
    }

    void image_callback(immutable_shared_ptr<sensor_msgs::msg::Image> image, immutable_shared_ptr<sensor_msgs::msg::CameraInfo> camera_info) {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::MONO8);
        cv::Mat camera_matrix(3, 3, CV_64F);
        camera_matrix.at<double>(0, 0) = camera_info->k.at(0);
        camera_matrix.at<double>(0, 1) = camera_info->k.at(1);
        camera_matrix.at<double>(0, 2) = camera_info->k.at(2);
        camera_matrix.at<double>(1, 0) = camera_info->k.at(3);
        camera_matrix.at<double>(1, 1) = camera_info->k.at(4);
        camera_matrix.at<double>(1, 2) = camera_info->k.at(5);
        camera_matrix.at<double>(2, 0) = camera_info->k.at(6);
        camera_matrix.at<double>(2, 1) = camera_info->k.at(7);
        camera_matrix.at<double>(2, 2) = camera_info->k.at(8);
        std::optional<apriltag::CameraLocalizationResult> result = _localizer->localize(
            cv_ptr->image,
            camera_matrix.reshape(1, {3, 3}),
            camera_info->d
        );
        if (result.has_value()) {
            RCLCPP_INFO_STREAM(get_logger(), "Got pose " << result->estimate.pose.affine());
            geometry_msgs::msg::PoseStamped camera_pose_msg;
            camera_pose_msg.pose = tf2::toMsg(result->estimate.pose);
            camera_pose_msg.header = image->header;
            _camera_pose_publisher->publish(camera_pose_msg);

            Eigen::Affine3d robot_to_camera = tf2::transformToEigen(_tf2_buffer->lookupTransform(
                "arducam1_optical", "base_link", image->header.stamp).transform);
            Eigen::Affine3d robot_to_world = result->estimate.pose * robot_to_camera;
            gtsam::Matrix6 camera_pose_covariance;
            camera_pose_covariance << 0.0625, 0, 0, 0, 0, 0,
                                      0, 0.0625, 0, 0, 0, 0,
                                      0, 0, 0.0625, 0, 0, 0,
                                      0, 0, 0, 0.0625, 0, 0,
                                      0, 0, 0, 0, 0.0625, 0,
                                      0, 0, 0, 0, 0, 0.0625;
            geometry_msgs::msg::PoseWithCovarianceStamped robot_pose_msg;
            robot_pose_msg.pose.pose = tf2::toMsg(robot_to_world);
            robot_pose_msg.pose.covariance = compute_robot_pose_covariance(
                camera_pose_covariance, robot_to_world);
            robot_pose_msg.header = image->header;
            robot_pose_msg.header.frame_id = "base_link";
            _pose_publisher->publish(robot_pose_msg);
        }
    }

    geometry_msgs::msg::PoseWithCovariance::_covariance_type compute_robot_pose_covariance(const gtsam::Matrix6& camera_pose_covariance, const Eigen::Affine3d robot_to_world) {
        gtsam::Matrix6 robot_pose_adjoint = gtsam::Pose3(robot_to_world.matrix()).AdjointMap();
        gtsam::Matrix6 robot_pose_covariance = robot_pose_adjoint * camera_pose_covariance * robot_pose_adjoint.transpose();

        geometry_msgs::msg::PoseWithCovariance::_covariance_type result;
        result.at(0) = robot_pose_covariance(0, 0);
        result.at(1) = robot_pose_covariance(0, 1);
        result.at(2) = robot_pose_covariance(0, 2);
        result.at(3) = robot_pose_covariance(0, 3);
        result.at(4) = robot_pose_covariance(0, 4);
        result.at(5) = robot_pose_covariance(0, 5);
        result.at(6) = robot_pose_covariance(1, 0);
        result.at(7) = robot_pose_covariance(1, 1);
        result.at(8) = robot_pose_covariance(1, 2);
        result.at(9) = robot_pose_covariance(1, 3);
        result.at(10) = robot_pose_covariance(1, 4);
        result.at(11) = robot_pose_covariance(1, 5);
        result.at(12) = robot_pose_covariance(2, 0);
        result.at(13) = robot_pose_covariance(2, 1);
        result.at(14) = robot_pose_covariance(2, 2);
        result.at(15) = robot_pose_covariance(2, 3);
        result.at(16) = robot_pose_covariance(2, 4);
        result.at(17) = robot_pose_covariance(2, 5);
        result.at(18) = robot_pose_covariance(3, 0);
        result.at(19) = robot_pose_covariance(3, 1);
        result.at(20) = robot_pose_covariance(3, 2);
        result.at(21) = robot_pose_covariance(3, 3);
        result.at(22) = robot_pose_covariance(3, 4);
        result.at(23) = robot_pose_covariance(3, 5);
        result.at(24) = robot_pose_covariance(4, 0);
        result.at(25) = robot_pose_covariance(4, 1);
        result.at(26) = robot_pose_covariance(4, 2);
        result.at(27) = robot_pose_covariance(4, 3);
        result.at(28) = robot_pose_covariance(4, 4);
        result.at(29) = robot_pose_covariance(4, 5);
        result.at(30) = robot_pose_covariance(5, 0);
        result.at(31) = robot_pose_covariance(5, 1);
        result.at(32) = robot_pose_covariance(5, 2);
        result.at(33) = robot_pose_covariance(5, 3);
        result.at(34) = robot_pose_covariance(5, 4);
        result.at(35) = robot_pose_covariance(5, 5);

        return result;
    }

};

int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AprilTagLocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
