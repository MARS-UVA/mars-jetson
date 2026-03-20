#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/synchronizer.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <apriltag_msgs/action/tare.hpp>

namespace {

std::optional<Eigen::Quaterniond> compute_average_rotation(std::vector<Eigen::Quaterniond> quats) {
    Eigen::Matrix<double, 4, 100> rotations;
    std::transform(
        quats.cbegin(),
        quats.cend(),
        rotations.colwise().begin(),
        [](const Eigen::Quaterniond& quat) { return quat.coeffs(); });
    Eigen::JacobiSVD<typeof(rotations)> svd;
    svd.compute(rotations, Eigen::ComputeFullU);
    if (svd.info() != Eigen::Success) {
        return std::nullopt;
    }
    return Eigen::Quaterniond(svd.matrixU().col(0));
}

}

namespace apriltag_localization {

class TaringServerNode : public rclcpp::Node {

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        geometry_msgs::msg::PoseWithCovarianceStamped,
        sensor_msgs::msg::Imu
    >;
    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
    using TareAction = apriltag_msgs::action::Tare;
    using TareGoalHandle = rclcpp_action::ServerGoalHandle<TareAction>;

    message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped> _camera_world_pose_subscriber;
    message_filters::Subscriber<sensor_msgs::msg::Imu> _imu_subscriber;
    std::shared_ptr<Synchronizer> _synchronizer;

    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr _set_rotation_publisher;

    rclcpp_action::Server<TareAction>::SharedPtr _tare_action_server;
    std::vector<Eigen::Quaterniond> _tare_samples;
    std::size_t _max_samples = 100;
    std::mutex _tare_samples_mutex;
    std::atomic_bool _taring;
    std::condition_variable _taring_cv;

public:
    TaringServerNode() : rclcpp::Node("taring_server") {
        using std::placeholders::_1, std::placeholders::_2;
        using namespace std::chrono_literals;
        declare_parameter<std::int64_t>("samples", 100);

        _camera_world_pose_subscriber.subscribe(
            this,
            "robot_pose_estimate",
            rclcpp::SensorDataQoS{}.get_rmw_qos_profile()
        );
        _imu_subscriber.subscribe(
            this,
            "/zed2i/zed_node/imu/data",
            rclcpp::QoS{10}.get_rmw_qos_profile()
        );

        _synchronizer = std::make_shared<Synchronizer>(
            SyncPolicy(10),
            _camera_world_pose_subscriber,
            _imu_subscriber
        );
        _synchronizer->setAgePenalty(0.5);
        _synchronizer->setMaxIntervalDuration(250ms);
        _synchronizer->registerCallback(
            std::bind(&TaringServerNode::on_pose_receive, this, _1, _2)
        );

        _set_rotation_publisher = create_publisher<geometry_msgs::msg::Quaternion>(
            "set_rotation",
            rclcpp::QoS{10}.reliable()
        );

        _tare_action_server = rclcpp_action::create_server<TareAction>(
            this,
            "tare",
            std::bind(&TaringServerNode::on_tare_receive, this, _1, _2),
            std::bind(&TaringServerNode::cancel_tare_request, this, _1),
            std::bind(&TaringServerNode::accept_tare_request, this, _1)
        );
    }

    rclcpp_action::GoalResponse on_tare_receive(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const TareAction::Goal> goal) {
        (void)uuid;
        (void)goal;
        RCLCPP_INFO(get_logger(), "Received taring request");
        if (!_taring) {
            std::int64_t max_samples;
            get_parameter<std::int64_t>("samples", max_samples);
            _max_samples = static_cast<std::size_t>(max_samples);
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        } else {
            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    void accept_tare_request(const std::shared_ptr<TareGoalHandle> goal_handle) {
        std::thread{std::bind(&TaringServerNode::tare, this, goal_handle)}.detach();
    }

    rclcpp_action::CancelResponse cancel_tare_request(const std::shared_ptr<TareGoalHandle> goal_handle) {
        (void)goal_handle;
        _taring_cv.notify_one();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void on_pose_receive(
        const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& camera_pose_msg,
        const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg
    ) {
        RCLCPP_INFO(get_logger(), "Entered on_pose_receive");
        if (_taring) {
            std::unique_lock<std::mutex> lock(_tare_samples_mutex, std::try_to_lock);
            if (lock.owns_lock()) {
                RCLCPP_INFO(get_logger(), "Got taring sample %ld", _tare_samples.size());
                Eigen::Affine3d camera_pose;
                tf2::fromMsg(camera_pose_msg->pose.pose, camera_pose);
                Eigen::Quaterniond imu_rotation;
                tf2::fromMsg(imu_msg->orientation, imu_rotation);
                _tare_samples.push_back(imu_rotation * Eigen::Quaterniond(camera_pose.linear()).inverse());
                if (_tare_samples.size() > _max_samples) {
                    lock.unlock();
                    _taring_cv.notify_one();
                }
            }
        }
    }

    void tare(const std::shared_ptr<TareGoalHandle> goal_handle) {
        using namespace std::chrono_literals;

        _set_rotation_publisher->publish(tf2::toMsg(Eigen::Quaterniond::Identity()));

        while (!_taring.exchange(true));
        RCLCPP_INFO(get_logger(), "Taring...");

        TareAction::Feedback::SharedPtr feedback = std::make_shared<TareAction::Feedback>();
        TareAction::Result::SharedPtr result = std::make_shared<TareAction::Result>();
        feedback->points_considered = 0;

        std::unique_lock<std::mutex> lock(_tare_samples_mutex);
        _tare_samples.clear();
        while (_tare_samples.size() < 50 && rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                goal_handle->canceled(result);
                while (_taring.exchange(false));
                RCLCPP_INFO(get_logger(), "Taring cancelled");
                return;
            }
            while (_tare_samples.size() <= feedback->points_considered) {
                _taring_cv.wait_for(lock, 1s);
                if (goal_handle->is_canceling()) {
                    goal_handle->canceled(result);
                    while (_taring.exchange(false));
                    RCLCPP_INFO(get_logger(), "Taring cancelled");
                    return;
                }
            }
            feedback->points_considered = _tare_samples.size();

            lock.unlock();
            goal_handle->publish_feedback(feedback);
            lock.lock();
        }

        if (rclcpp::ok()) {
            std::optional<Eigen::Quaterniond> tare_rotation = compute_average_rotation(_tare_samples);
            if (tare_rotation.has_value()) {
                result->tared_rotation = tf2::toMsg(tare_rotation.value());
                goal_handle->succeed(result);
                _set_rotation_publisher->publish(result->tared_rotation);
                RCLCPP_INFO(get_logger(), "Taring success!");
            } else {
                goal_handle->abort(result);
                RCLCPP_ERROR(get_logger(), "Failed to compute tare rotation");
            }
        }
        while (_taring.exchange(false));
    }

};

}

int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<apriltag_localization::TaringServerNode>());
    rclcpp::shutdown();
    return 0;
}
