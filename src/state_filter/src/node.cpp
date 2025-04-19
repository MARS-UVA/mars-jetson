#include <chrono>
#include <functional>
#include <memory>
#include <queue>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
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

    std::queue<sensor_msgs::msg::Imu> _imu_message_queue;
    std::queue<geometry_msgs::msg::PoseWithCovarianceStamped> _apriltag_pose_message_queue;

    std::unique_ptr<filter::RobotModelFilter> _filter;

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
        _update_timer = create_wall_timer(20ms, std::bind(&RobotUkfNode::update_filter, this));
        _last_update_time = get_clock()->now();
        _filter = std::make_unique<filter::RobotModelFilter>(filter::State::Zero(),
                                                             filter::StateCovariance::Identity(),
                                                             std::bind(&RobotUkfNode::update_state, this, _1, _2),
                                                             std::bind(&RobotUkfNode::apriltag_state_to_observation, this, _1),
                                                             std::bind(&RobotUkfNode::gyro_state_to_observation, this, _1),
                                                             0.1,
                                                             2.0,
                                                             0.0);
    }

    void update_filter() {
        double dt = (get_clock()->now() - _last_update_time).seconds();
        _last_update_time = get_clock()->now();
        RCLCPP_DEBUG(get_logger(), "Time since last update: %.2f ms", dt * 1000);
        RCLCPP_DEBUG(get_logger(), "Received %lu AprilTag updates and %lu IMU updates", _apriltag_pose_message_queue.size(), _imu_message_queue.size());
        _filter->predict(dt, process_model_covariance(dt));

        sensor_msgs::msg::Imu imu_data;
        geometry_msgs::msg::PoseWithCovarianceStamped apriltag_pose_data;
        rclcpp::Time imu_data_time;
        rclcpp::Time apriltag_pose_data_time;
        while (!_imu_message_queue.empty() && !_apriltag_pose_message_queue.empty()) {
            imu_data = _imu_message_queue.front();
            apriltag_pose_data = _apriltag_pose_message_queue.front();
            imu_data_time = imu_data.header.stamp;
            apriltag_pose_data_time = apriltag_pose_data.header.stamp;
            if (imu_data_time < apriltag_pose_data_time) {
                _imu_message_queue.pop();
                _filter->update_from_gyro(get_imu_observation(imu_data), get_imu_observation_covariance(imu_data));
            } else {
                _apriltag_pose_message_queue.pop();
                _filter->update_from_apriltag(get_apriltag_observation(apriltag_pose_data), get_apriltag_observation_covariance(apriltag_pose_data));
            }
        }
        while (!_imu_message_queue.empty()) {
            imu_data = _imu_message_queue.front();
            _imu_message_queue.pop();
            _filter->update_from_gyro(get_imu_observation(imu_data), get_imu_observation_covariance(imu_data));
        }
        while (!_apriltag_pose_message_queue.empty()) {
            apriltag_pose_data = _apriltag_pose_message_queue.front();
            _apriltag_pose_message_queue.pop();
            _filter->update_from_apriltag(get_apriltag_observation(apriltag_pose_data), get_apriltag_observation_covariance(apriltag_pose_data));
        }

        publish_transform();
    }

    void publish_transform() {
        const filter::State& state = _filter->state_estimate();

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = get_clock()->now();
        transform.header.frame_id = "world";
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

        RCLCPP_DEBUG(get_logger(), "Publishing filtered pose: x = %.3f m, y = %.3f m, theta = %.4f rad", state(filter::STATE_X), state(filter::STATE_Y), state(filter::STATE_THETA));
        _filtered_pose_broadcaster->sendTransform(transform);
    }

    void on_imu_data(const sensor_msgs::msg::Imu imu_data) {
        _imu_message_queue.emplace(imu_data);
    }

    void on_apriltag_pose_data(const geometry_msgs::msg::PoseWithCovarianceStamped apriltag_pose_data) {
        _apriltag_pose_message_queue.emplace(apriltag_pose_data);
    }

    filter::AprilTagObservation get_apriltag_observation(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_data) {
        tf2::Quaternion orientation;
        tf2::fromMsg(pose_data.pose.pose.orientation, orientation);
        tf2::Matrix3x3 orientation_matrix(orientation);
        double roll, pitch, yaw;
        orientation_matrix.getRPY(roll, pitch, yaw);
        filter::AprilTagObservation result;
        result << pose_data.pose.pose.position.x, pose_data.pose.pose.position.y, yaw;
        return result;
    }

    filter::AprilTagObservationCovariance get_apriltag_observation_covariance(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_data) {
        filter::AprilTagObservationCovariance result;
        result << pose_data.pose.covariance[0], pose_data.pose.covariance[1], pose_data.pose.covariance[5],
                  pose_data.pose.covariance[6], pose_data.pose.covariance[7], pose_data.pose.covariance[11],
                  pose_data.pose.covariance[30], pose_data.pose.covariance[31], pose_data.pose.covariance[35];
        return result;
    }

    filter::GyroObservation get_imu_observation(const sensor_msgs::msg::Imu& imu_data) {
        filter::GyroObservation result;
        result << imu_data.angular_velocity.z, imu_data.linear_acceleration.x, imu_data.linear_acceleration.y;
        return result;
    }

    filter::GyroObservationCovariance get_imu_observation_covariance(const sensor_msgs::msg::Imu& imu_data) {
        Eigen::Matrix<double, 1, 1> angular_velocity_part = Eigen::Matrix<double, 1, 1>::Zero();
        if (imu_data.angular_velocity_covariance[0] != -1) {
            angular_velocity_part << imu_data.angular_velocity_covariance[8];
        }
        Eigen::Matrix<double, 2, 2> linear_acceleration_part = Eigen::Matrix<double, 2, 2>::Zero();
        if (imu_data.linear_acceleration_covariance[0] != -1) {
            linear_acceleration_part << imu_data.linear_acceleration_covariance[0], imu_data.linear_acceleration_covariance[1],
                                        imu_data.linear_acceleration_covariance[3], imu_data.linear_acceleration_covariance[4];
        }
        filter::GyroObservationCovariance result = Eigen::Matrix<double, 3, 3>::Zero();
        result.topLeftCorner<1, 1>() = angular_velocity_part;
        result.bottomRightCorner<2, 2>() = linear_acceleration_part;
        return result;
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
        h << 0, 0, 0, 0, 0, 0, 0, 1, 0,
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
            std::cerr << "WARNING: Eigendecomposition solver for continuous white noise matrix failed.\n";
        }
        return eigensolver.operatorSqrt();
    }

    filter::StateCovariance process_model_covariance(const double& dt) {
        filter::StateCovariance process_model_covariance;
        process_model_covariance.topLeftCorner<3, 3>() = continuous_white_noise_sqrt(dt);
        process_model_covariance.block<3, 3>(3, 3) = process_model_covariance.topLeftCorner<3, 3>();
        process_model_covariance.bottomRightCorner<3, 3>() = process_model_covariance.topLeftCorner<3, 3>();
        process_model_covariance *= 0.1;
        return process_model_covariance;
    }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotUkfNode>());
    rclcpp::shutdown();
    return 0;
}