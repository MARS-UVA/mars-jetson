#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <memory>
#include <queue>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "state_filter/filter.hpp"

using namespace std::chrono_literals;

class RobotUkfNode : public rclcpp::Node {

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_subscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _apriltag_pose_subscription;

    std::unique_ptr<tf2_ros::TransformBroadcaster> _filtered_pose_broadcaster;

    rclcpp::TimerBase::SharedPtr _update_timer;

    std::optional<sensor_msgs::msg::Imu> _imu_message;
    std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> _apriltag_pose_message;

    std::unique_ptr<filter::RobotModelFilter> _filter;

    std::vector<double> _gyro_angle_displacements;
    Eigen::Rotation2D<double> _gyro_angle_correction = Eigen::Rotation2D<double>::Identity();
    bool _ready_for_gyro_inclusion = false;

    rclcpp::Time _last_update_time;

public:

    RobotUkfNode() : rclcpp::Node("robot_ukf") {
        using std::placeholders::_1;
        using std::placeholders::_2;
        _imu_subscription = create_subscription<sensor_msgs::msg::Imu>(
            "/localization/imu", 10, std::bind(&RobotUkfNode::on_imu_data, this, _1));
        _apriltag_pose_subscription = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/localization/estimated_pose", 10, std::bind(&RobotUkfNode::on_apriltag_pose_data, this, _1));
        _filtered_pose_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        _update_timer = create_timer(20ms, std::bind(&RobotUkfNode::update_filter, this));
        _last_update_time = get_clock()->now();
        _filter = std::make_unique<filter::RobotModelFilter>(filter::State::Zero(),
                                                             filter::StateCovariance::Identity(),
                                                             std::bind(&RobotUkfNode::update_state, this, _1, _2),
                                                             std::bind(&RobotUkfNode::apriltag_state_to_observation, this, _1),
                                                             std::bind(&RobotUkfNode::gyro_state_to_observation, this, _1),
                                                             0.1,
                                                             2.0,
                                                             0.0,
                                                             get_logger());

        publish_transform();

        RCLCPP_INFO(get_logger(), "Note: will not incorporate IMU measurements into filter until angle offset is calcualated");
    }

    void update_filter() {
        rclcpp::Time now = get_clock()->now();
        double dt = (now - _last_update_time).seconds();
        RCLCPP_DEBUG(get_logger(), "Time since last update: %.2f ms", dt * 1000);
        _last_update_time = now;
        _filter->predict(dt, process_model_covariance(dt));

        filter::GyroObservation gyro_observation;
        filter::AprilTagObservation apriltag_observation;
        if (_apriltag_pose_message.has_value()) {
            apriltag_observation = get_apriltag_observation(_apriltag_pose_message.value());
            _filter->update_from_apriltag(apriltag_observation, get_apriltag_observation_covariance(_apriltag_pose_message.value()));
            _apriltag_pose_message = std::nullopt;
            if (_imu_message.has_value()) {
                gyro_observation = get_imu_observation(_imu_message.value());
                if (_ready_for_gyro_inclusion) {
                    _filter->update_from_gyro(gyro_observation, get_imu_observation_covariance(_imu_message.value()));
                    _imu_message = std::nullopt;
                } else {
                    double offset = gyro_observation(filter::GYRO_THETA) - apriltag_observation(filter::APRILTAG_THETA);
                    filter::normalize_angle<double>(offset);
                    _gyro_angle_displacements.push_back(offset);
                    if (_gyro_angle_displacements.size() >= 20) {
                        // Circular mean
                        double sines;
                        double cosines;
                        for (auto it = _gyro_angle_displacements.begin(); it < _gyro_angle_displacements.end(); it++) {
                            sines += std::sin(*it);
                            cosines += std::cos(*it);
                        }
                        _gyro_angle_correction = Eigen::Rotation2D(-std::atan2(sines, cosines));
                        _ready_for_gyro_inclusion = true;
                        _gyro_angle_displacements.clear();
                        RCLCPP_INFO(get_logger(), "Gyro axes will be rotated by %.4f rad to correct orientation difference", _gyro_angle_correction.angle());
                    }
                }
            }
        }
        if (_imu_message.has_value() && _ready_for_gyro_inclusion) {
            gyro_observation = get_imu_observation(_imu_message.value());
            _filter->update_from_gyro(gyro_observation, get_imu_observation_covariance(_imu_message.value()));
            _imu_message = std::nullopt;
        }

        const filter::State& state = _filter->state_estimate();
        RCLCPP_DEBUG(get_logger(), "State: x = %.3f m, x' = %.3f m/s, x'' = %.3f m/s^2, y = %.3f, y' = %.3f m/s, y'' = %.3f m/s^2, theta = %.4f rad, theta' = %.4f rad/s, theta'' = %.4f rad/s^2",
            state(filter::STATE_X), state(filter::STATE_DX), state(filter::STATE_DDX),
            state(filter::STATE_Y), state(filter::STATE_DY), state(filter::STATE_DDY),
            state(filter::STATE_THETA), state(filter::STATE_DTHETA), state(filter::STATE_DDTHETA));

        publish_transform();
    }

    void publish_transform() {
        const filter::State& state = _filter->state_estimate();

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = get_clock()->now();
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";

        transform.transform.translation.x = state(filter::STATE_X);
        transform.transform.translation.y = state(filter::STATE_Y);
        transform.transform.translation.z = 0.0;

        tf2::Quaternion quat;
        quat.setRPY(0, 0, state(filter::STATE_THETA));
        transform.transform.rotation.x = quat.x();
        transform.transform.rotation.y = quat.y();
        transform.transform.rotation.z = quat.z();
        transform.transform.rotation.w = quat.w();

        RCLCPP_INFO(get_logger(), "Publishing filtered pose: x = %.3f m, y = %.3f m, theta = %.4f rad", state(filter::STATE_X), state(filter::STATE_Y), state(filter::STATE_THETA));
        _filtered_pose_broadcaster->sendTransform(transform);
    }

    void on_imu_data(const sensor_msgs::msg::Imu imu_data) {
        _imu_message = std::make_optional(imu_data);
    }

    void on_apriltag_pose_data(const geometry_msgs::msg::PoseWithCovarianceStamped apriltag_pose_data) {
        RCLCPP_INFO(get_logger(), "Got AprilTag pose");
        _apriltag_pose_message = std::make_optional(apriltag_pose_data);
        // if (apriltag_pose_data.pose.covariance[0] < 4) {
        //     RCLCPP_INFO(get_logger(), "Got AprilTag pose");
        //     _apriltag_pose_message = std::make_optional(apriltag_pose_data);
        // }
    }

    filter::AprilTagObservation get_apriltag_observation(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_data) {
        tf2::Quaternion orientation;
        tf2::fromMsg(pose_data.pose.pose.orientation, orientation);
        tf2::Matrix3x3 orientation_matrix(orientation);
        double roll, pitch, yaw;
        orientation_matrix.getRPY(roll, pitch, yaw);
        filter::AprilTagObservation result;
        result << pose_data.pose.pose.position.x, pose_data.pose.pose.position.y, yaw;
        RCLCPP_DEBUG(get_logger(), "Considering AprilTag pose data: x = %.3f m, y = %.3f m, theta = %.4f rad", pose_data.pose.pose.position.x, pose_data.pose.pose.position.y, yaw);
        return result;
    }

    filter::AprilTagObservationCovariance get_apriltag_observation_covariance(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_data) {
        filter::AprilTagObservationCovariance result;
        result << pose_data.pose.covariance[0], pose_data.pose.covariance[1], pose_data.pose.covariance[5],
                  pose_data.pose.covariance[6], pose_data.pose.covariance[7], pose_data.pose.covariance[11],
                  pose_data.pose.covariance[30], pose_data.pose.covariance[31], pose_data.pose.covariance[35];
        Eigen::SelfAdjointEigenSolver<decltype(result)> eigensolver;
        eigensolver.compute(result);
        if (eigensolver.info() != Eigen::Success) {
            RCLCPP_WARN(get_logger(), "Eigendecomposition solver for AprilTag observation covariance failed");
        }
        return eigensolver.operatorSqrt();
    }

    filter::GyroObservation get_imu_observation(const sensor_msgs::msg::Imu& imu_data) {
        Eigen::Quaternion<double> orientation = {
            imu_data.orientation.w, imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z
        };
        double yaw = orientation.toRotationMatrix().eulerAngles(0, 1, 2).z();
        Eigen::Vector3d angular_velocity { imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z };
        Eigen::Vector3d linear_acceleration { imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z };
        angular_velocity = orientation * angular_velocity;
        linear_acceleration = orientation * linear_acceleration;
        filter::GyroObservation result;
        result(filter::GYRO_THETA) = _gyro_angle_correction.smallestAngle() + yaw;
        result(filter::GYRO_DTHETA) = angular_velocity.z();
        result(filter::GYRO_DDX) = linear_acceleration.x();
        result(filter::GYRO_DDY) = linear_acceleration.y();
        result.tail<2>().applyOnTheLeft(_gyro_angle_correction.toRotationMatrix());
        filter::normalize_angles<double, filter::GYRO_VARS, 1, filter::GYRO_THETA>(result);
        RCLCPP_DEBUG(get_logger(), "Considering IMU data: theta = %.4f rad, theta' = %.4f rad/s, x'' = %.3f m/s^2, y'' = %.3f m/s^2", result(filter::GYRO_THETA), result(filter::GYRO_DTHETA), result(filter::GYRO_DDX), result(filter::GYRO_DDY));
        return result;
    }

    filter::GyroObservationCovariance get_imu_observation_covariance(const sensor_msgs::msg::Imu& imu_data) {
        Eigen::Matrix<double, 1, 1> orientation_part = Eigen::Matrix<double, 1, 1>::Zero();
        if (imu_data.orientation_covariance[0] != -1) {
            orientation_part << imu_data.orientation_covariance[8];
        }
        Eigen::Matrix<double, 1, 1> angular_velocity_part = Eigen::Matrix<double, 1, 1>::Zero();
        if (imu_data.angular_velocity_covariance[0] != -1) {
            angular_velocity_part << imu_data.angular_velocity_covariance[8];
        }
        Eigen::Matrix<double, 2, 2> linear_acceleration_part = Eigen::Matrix<double, 2, 2>::Zero();
        if (imu_data.linear_acceleration_covariance[0] != -1) {
            linear_acceleration_part << imu_data.linear_acceleration_covariance[0], imu_data.linear_acceleration_covariance[1],
                                        imu_data.linear_acceleration_covariance[3], imu_data.linear_acceleration_covariance[4];
        }
        filter::GyroObservationCovariance result = filter::GyroObservationCovariance::Zero();
        result.topLeftCorner<1, 1>() = orientation_part;
        result.block<1, 1>(1, 1) = angular_velocity_part;
        result.bottomRightCorner<2, 2>() = _gyro_angle_correction * linear_acceleration_part * _gyro_angle_correction.inverse();
        Eigen::SelfAdjointEigenSolver<decltype(result)> eigensolver;
        eigensolver.compute(result);
        if (eigensolver.info() != Eigen::Success) {
            RCLCPP_WARN(get_logger(), "Eigendecomposition solver for gyro observation covariance failed");
        }
        return eigensolver.operatorSqrt();
    }

    void update_state(Eigen::Ref<filter::State> state, const double& dt) {
        state(filter::STATE_X) += state(filter::STATE_DX) * dt;
        state(filter::STATE_DX) += state(filter::STATE_DDX) * dt;
        state(filter::STATE_Y) += state(filter::STATE_DY) * dt + 0.5 * state(filter::STATE_DDY) * dt * dt;
        state(filter::STATE_DY) += state(filter::STATE_DDY) * dt;
        state(filter::STATE_THETA) += state(filter::STATE_DTHETA) * dt + 0.5 * state(filter::STATE_DDTHETA) * dt * dt;
        state(filter::STATE_DTHETA) += state(filter::STATE_DDTHETA) * dt;
        filter::normalize_angles<double, filter::STATE_VARS, 1, filter::STATE_THETA>(state);
    }

    filter::AprilTagObservation apriltag_state_to_observation(const filter::State& state) {
        Eigen::Matrix<double, filter::APRILTAG_VARS, filter::STATE_VARS> h;
        h << 1, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 1, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 1, 0, 0;
        return h * state;
    }

    filter::GyroObservation gyro_state_to_observation(const filter::State& state) {
        Eigen::Matrix<double, filter::GYRO_VARS, filter::STATE_VARS> h;
        h << 0, 0, 0, 0, 0, 0, 1, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 1, 0,
             0, 0, 1, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 1, 0, 0, 0;
        return h * state;
    }

    Eigen::Matrix<double, 3, 3> continuous_white_noise_sqrt(const double& dt) {
        Eigen::Matrix<double, 3, 3> result;
        result << std::pow(dt, 5) / 20, std::pow(dt, 4) / 8, std::pow(dt, 3) / 6,
                   std::pow(dt, 4) / 8, std::pow(dt, 3) / 3,         dt * dt / 2,
                   std::pow(dt, 3) / 6,         dt * dt / 2,                  dt;
        Eigen::SelfAdjointEigenSolver<decltype(result)> eigensolver;
        eigensolver.compute(result);
        if (eigensolver.info() != Eigen::Success) {
            RCLCPP_WARN(get_logger(), "Eigendecomposition solver for continuous white noise matrix failed");
        }
        return eigensolver.operatorSqrt();
    }

    filter::StateCovariance process_model_covariance(const double& dt) {
        filter::StateCovariance process_model_covariance;
        process_model_covariance.topLeftCorner<3, 3>() = continuous_white_noise_sqrt(dt);
        process_model_covariance.block<3, 3>(3, 3) = process_model_covariance.topLeftCorner<3, 3>();
        process_model_covariance.bottomRightCorner<3, 3>() = process_model_covariance.topLeftCorner<3, 3>();
        process_model_covariance *= 0.5;
        return process_model_covariance;
    }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotUkfNode>());
    rclcpp::shutdown();
    return 0;
}