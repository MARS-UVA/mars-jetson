// Nav2 plugin wrapper around PointAndTurnCore. Owns what needs ROS:
// parameters, TF, costmap collision checks and debug topics. The decisions
// about how to move live in the core.
#ifndef POINT_AND_TURN_CONTROLLER__POINT_AND_TURN_CONTROLLER_HPP_
#define POINT_AND_TURN_CONTROLLER__POINT_AND_TURN_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"

#include "point_and_turn_controller/core.hpp"

namespace point_and_turn_controller
{

class PointAndTurnController : public nav2_core::Controller
{
public:
  PointAndTurnController() = default;
  ~PointAndTurnController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  void reset() override;

private:
  // Throws NoValidControl when the next motion would put the footprint on a
  // lethal cell. pose is in the costmap frame.
  void checkCollision(
    const geometry_msgs::msg::PoseStamped & pose, const Command & command,
    double measured_speed);
  bool inCollision(
    double x, double y, double yaw, const std::vector<geometry_msgs::msg::Point> & footprint);
  void publishDebug();

  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("PointAndTurnController")};
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> collision_checker_;

  // setSpeedLimit arrives on the speed-limit subscription's thread while
  // computeVelocityCommands runs on the action thread; this guards core_.
  std::mutex mutex_;
  std::unique_ptr<PointAndTurnCore> core_;
  std::string plan_frame_;

  double transform_tolerance_{0.2};
  double collision_margin_{0.15};
  double rotation_check_step_{0.1};
  bool allow_unknown_{true};

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr vertices_pub_;
  bool state_published_{false};
  State published_state_{State::IDLE};
  std::size_t published_revision_{0};
};

}  // namespace point_and_turn_controller

#endif  // POINT_AND_TURN_CONTROLLER__POINT_AND_TURN_CONTROLLER_HPP_
