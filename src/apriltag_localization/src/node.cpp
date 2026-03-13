#include <atomic>
#include <chrono>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>
#include <tagStandard41h12.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
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
#include <apriltag_msgs/action/tare.hpp>

#include "field.hpp"
#include "localizer.hpp"

template <typename T>
using immutable_shared_ptr = const std::shared_ptr<const T>;

namespace apriltag_localization {

class AprilTagLocalizationNode : public rclcpp::Node {

    using TareAction = apriltag_msgs::action::Tare;
    using TareGoalHandle = rclcpp_action::ServerGoalHandle<TareAction>;

    std::unique_ptr<apriltag::CameraLocalizer> _localizer;
    image_transport::CameraSubscriber _camera_subscriber;
    std::unique_ptr<tf2_ros::Buffer> _tf2_buffer;
    std::shared_ptr<tf2_ros::TransformListener> _tf2_listener;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _camera_pose_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _pose_publisher;

    rclcpp_action::Server<TareAction>::SharedPtr _tare_action_server;
    std::vector<Eigen::Affine3d> _tare_points;
    std::mutex _tare_points_mutex;
    std::atomic_bool _taring;
    Eigen::Affine3d _tare_transform = Eigen::Affine3d::Identity();
    std::condition_variable _taring_cv;

public:
    AprilTagLocalizationNode() : rclcpp::Node("apriltag_localization") {
        using std::placeholders::_1, std::placeholders::_2;
        declare_parameter<std::string>("field");

        std::string field_path;
        get_parameter<std::string>("field", field_path);
        std::shared_ptr<apriltag::AprilTagField> field = apriltag::AprilTagField::parse(std::ifstream(field_path));
        _localizer = std::make_unique<apriltag::CameraLocalizer>(field, apriltag::PnPMethod::SQPNP);
        _localizer->detector().nthreads() = 8;

        _camera_subscriber = image_transport::create_camera_subscription(
            this,
            "image_raw",
            std::bind(&AprilTagLocalizationNode::on_image_receive, this, _1, _2),
            "raw",
            rclcpp::SensorDataQoS().get_rmw_qos_profile()
        );
        _tf2_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        _tf2_listener = std::make_shared<tf2_ros::TransformListener>(*_tf2_buffer);
        _camera_pose_publisher = create_publisher<geometry_msgs::msg::PoseStamped>("camera_pose_estimate", rclcpp::SensorDataQoS{});
        _pose_publisher = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("robot_pose_estimate", rclcpp::SensorDataQoS{});

        _tare_action_server = rclcpp_action::create_server<TareAction>(
            this,
            "tare",
            std::bind(&AprilTagLocalizationNode::on_tare_receive, this, _1, _2),
            std::bind(&AprilTagLocalizationNode::cancel_tare_request, this, _1),
            std::bind(&AprilTagLocalizationNode::accept_tare_request, this, _1)
        );
    }

    rclcpp_action::GoalResponse on_tare_receive(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const TareAction::Goal> goal) {
        (void)uuid;
        (void)goal;
        RCLCPP_INFO(get_logger(), "Received taring request");
        return _taring ? rclcpp_action::GoalResponse::REJECT : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    void accept_tare_request(const std::shared_ptr<TareGoalHandle> goal_handle) {
        std::thread{std::bind(&AprilTagLocalizationNode::tare, this, goal_handle)}.detach();
    }

    rclcpp_action::CancelResponse cancel_tare_request(const std::shared_ptr<TareGoalHandle> goal_handle) {
        (void)goal_handle;
        _taring_cv.notify_one();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void on_image_receive(immutable_shared_ptr<sensor_msgs::msg::Image> image, immutable_shared_ptr<sensor_msgs::msg::CameraInfo> camera_info) {
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
            Eigen::Affine3d camera_to_world = _tare_transform * result->estimate.pose;
            RCLCPP_INFO_STREAM(get_logger(), "Got pose " << (camera_to_world).affine());

            if (_taring.load()) {
                std::unique_lock<std::mutex> lock(_tare_points_mutex, std::try_to_lock);
                if (lock.owns_lock()) {
                    _tare_points.push_back(camera_to_world);
                    lock.unlock();
                    _taring_cv.notify_one();
                    return;
                }
            } else {
                geometry_msgs::msg::PoseStamped camera_pose_msg;
                camera_pose_msg.pose = tf2::toMsg(camera_to_world);
                camera_pose_msg.header = image->header;
                _camera_pose_publisher->publish(camera_pose_msg);

                Eigen::Affine3d robot_to_camera = tf2::transformToEigen(_tf2_buffer->lookupTransform(
                    "arducam1_optical", "base_link", image->header.stamp).transform);
                Eigen::Affine3d robot_to_world = camera_to_world * robot_to_camera;
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
    }

    void tare(const std::shared_ptr<TareGoalHandle> goal_handle) {
        using namespace std::chrono_literals;

        while (!_taring.exchange(true));
        RCLCPP_INFO(get_logger(), "Taring...");

        TareAction::Feedback::SharedPtr feedback = std::make_shared<TareAction::Feedback>();
        TareAction::Result::SharedPtr result = std::make_shared<TareAction::Result>();
        feedback->points_considered = 0;

        std::unique_lock<std::mutex> lock(_tare_points_mutex);
        _tare_points.clear();
        while (_tare_points.size() < 50 && rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                goal_handle->canceled(result);
                while (_taring.exchange(false));
                RCLCPP_INFO(get_logger(), "Taring cancelled");
                return;
            }
            while (_tare_points.size() <= feedback->points_considered) {
                _taring_cv.wait(lock);
                if (goal_handle->is_canceling()) {
                    goal_handle->canceled(result);
                    while (_taring.exchange(false));
                    RCLCPP_INFO(get_logger(), "Taring cancelled");
                    return;
                }
            }
            feedback->points_considered = _tare_points.size();

            lock.unlock();
            goal_handle->publish_feedback(feedback);
            std::this_thread::sleep_for(200ms);
            lock.lock();
        }

        if (rclcpp::ok()) {
            std::optional<Eigen::Affine3d> tare_transform = compute_tare_transform();
            if (tare_transform.has_value()) {
                _tare_transform = tare_transform.value();
                result->zero_pose = tf2::toMsg(tare_transform.value());
                goal_handle->succeed(result);
                RCLCPP_INFO(get_logger(), "Taring success!");
            } else {
                goal_handle->abort(result);
                RCLCPP_ERROR(get_logger(), "Failed to compute tare transform");
            }
        }
        while (_taring.exchange(false));
    }

    std::optional<Eigen::Affine3d> compute_tare_transform() {
        Eigen::Matrix<double, 3, 100> translations;
        std::transform(
            _tare_points.cbegin(),
            _tare_points.cend(),
            translations.colwise().begin(),
            [](const Eigen::Affine3d& transform) { return transform.translation(); });
        Eigen::Matrix<double, 4, 100> rotations;
        std::transform(
            _tare_points.cbegin(),
            _tare_points.cend(),
            rotations.colwise().begin(),
            [](const Eigen::Affine3d& transform) { return Eigen::Quaterniond(transform.linear()).coeffs(); });

        Eigen::Affine3d zero_transform;
        zero_transform.translation() = translations.rowwise().mean();
        Eigen::JacobiSVD<typeof(rotations)> svd;
        svd.compute(rotations, Eigen::ComputeThinU);
        if (svd.info() != Eigen::Success) {
            return std::nullopt;
        }
        zero_transform.linear() = Eigen::Quaterniond(svd.matrixU().col(0)).matrix();
        return zero_transform.inverse();
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

}

int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<apriltag_localization::AprilTagLocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
