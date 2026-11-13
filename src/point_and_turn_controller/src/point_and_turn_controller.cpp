#include "point_and_turn_controller/point_and_turn_controller.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace point_and_turn_controller
{

namespace
{

// Yaw straight from the quaternion; avoids pulling in tf2_geometry_msgs for
// one conversion.
double yawOf(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

void PointAndTurnController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::ControllerException("Unable to lock the controller server node");
  }
  name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  collision_checker_.setCostmap(costmap_ros_->getCostmap());

  // Declare-then-read keeps the default in one place: the struct initialiser.
  auto parameter = [&](const std::string & key, auto & value) {
      nav2_util::declare_parameter_if_not_declared(
        node, name_ + "." + key, rclcpp::ParameterValue(value));
      node->get_parameter(name_ + "." + key, value);
    };
  Params p;
  parameter("max_linear_speed", p.max_linear_speed);
  parameter("min_linear_speed", p.min_linear_speed);
  parameter("linear_decel", p.linear_decel);
  parameter("max_angular_speed", p.max_angular_speed);
  parameter("min_angular_speed", p.min_angular_speed);
  parameter("angular_gain", p.angular_gain);
  parameter("align_tolerance", p.align_tolerance);
  parameter("realign_tolerance", p.realign_tolerance);
  parameter("max_cross_track_error", p.max_cross_track_error);
  parameter("min_bearing_distance", p.min_bearing_distance);
  parameter("segment_arrival_tolerance", p.segment_arrival_tolerance);
  parameter("goal_arrival_tolerance", p.goal_arrival_tolerance);
  parameter("final_align_tolerance", p.final_align_tolerance);
  parameter("replan_match_tolerance", p.replan_match_tolerance);
  parameter("drive_heading_gain", p.drive_heading_gain);
  parameter("max_drive_angular_speed", p.max_drive_angular_speed);
  parameter("allow_reversing", p.allow_reversing);
  parameter("corridor_tolerance", p.simplifier.corridor_tolerance);
  parameter("min_segment_length", p.simplifier.min_segment_length);
  parameter("transform_tolerance", transform_tolerance_);
  parameter("collision_margin", collision_margin_);
  parameter("rotation_check_step", rotation_check_step_);
  parameter("allow_unknown", allow_unknown_);

  // Fail configure, and so the controller server's activation, on a bad
  // value rather than discovering it as odd motion later.
  std::string error = validate(p);
  if (error.empty() && !(transform_tolerance_ > 0.0)) {
    error = "transform_tolerance must be > 0";
  }
  if (error.empty() && !(collision_margin_ >= 0.0)) {
    error = "collision_margin must be >= 0";
  }
  if (error.empty() && !(rotation_check_step_ > 0.0)) {
    error = "rotation_check_step must be > 0";
  }
  if (!error.empty()) {
    throw nav2_core::InvalidController(name_ + ": " + error);
  }
  core_ = std::make_unique<PointAndTurnCore>(p);

  // Transient local, so a late `ros2 topic echo` or RViz still sees the
  // current state and vertices instead of waiting for the next change.
  const auto latched = rclcpp::QoS(1).transient_local();
  state_pub_ = node->create_publisher<std_msgs::msg::String>(name_ + "/state", latched);
  vertices_pub_ = node->create_publisher<nav_msgs::msg::Path>(name_ + "/vertices", latched);

  RCLCPP_INFO(
    logger_, "Configured %s: v_max %.2f m/s, w_max %.2f rad/s, reversing %s", name_.c_str(),
    p.max_linear_speed, p.max_angular_speed, p.allow_reversing ? "on" : "off");
}

void PointAndTurnController::cleanup()
{
  state_pub_.reset();
  vertices_pub_.reset();
  core_.reset();
}

void PointAndTurnController::activate()
{
  state_pub_->on_activate();
  vertices_pub_->on_activate();
}

void PointAndTurnController::deactivate()
{
  state_pub_->on_deactivate();
  vertices_pub_->on_deactivate();
}

void PointAndTurnController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(mutex_);
  plan_frame_ = path.header.frame_id;
  std::vector<Pose2D> plan;
  plan.reserve(path.poses.size());
  for (const auto & pose : path.poses) {
    plan.push_back({pose.pose.position.x, pose.pose.position.y, yawOf(pose.pose.orientation)});
  }
  // No robot pose here, so the core only stores it; the next
  // computeVelocityCommands applies it.
  core_->setPlan(plan);
}

geometry_msgs::msg::TwistStamped PointAndTurnController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!core_->hasPlan()) {
    throw nav2_core::InvalidPath(name_ + ": no plan received");
  }
  // The state machine runs in the plan frame (map). Transforming the one
  // robot pose each cycle is O(1); transforming the plan into the costmap
  // frame would be O(n) every cycle, and map -> odom moves under the plan
  // anyway whenever RTAB-Map corrects.
  geometry_msgs::msg::PoseStamped robot;
  if (!nav2_util::transformPoseInTargetFrame(
      pose, robot, *tf_, plan_frame_, transform_tolerance_))
  {
    throw nav2_core::ControllerTFError(
            name_ + ": cannot transform the robot pose into " + plan_frame_);
  }
  const Command command = core_->update(
    {robot.pose.position.x, robot.pose.position.y, yawOf(robot.pose.orientation)});
  publishDebug();
  // Collision checks use the costmap frame pose that was passed in, so no
  // costmap cell ever needs transforming.
  checkCollision(pose, command, velocity.linear.x);

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = command.linear;
  cmd_vel.twist.angular.z = command.angular;
  return cmd_vel;
}

void PointAndTurnController::checkCollision(
  const geometry_msgs::msg::PoseStamped & pose, const Command & command, double measured_speed)
{
  const State state = core_->state();
  const double x = pose.pose.position.x;
  const double y = pose.pose.position.y;
  const double yaw = yawOf(pose.pose.orientation);
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  // Hold the costmap still while reading it; its update thread writes it.
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());
  const std::vector<geometry_msgs::msg::Point> footprint = costmap_ros_->getRobotFootprint();

  if (state == State::DRIVE && command.linear != 0.0) {
    // Look as far as the robot needs to stop, from the faster of commanded
    // and measured speed (it may still be carrying speed from before), plus
    // a margin for latency. Sample 0 is skipped: the voxel layer clears the
    // current footprint, and a robot already overlapping a lethal cell can't
    // escape by refusing every command.
    const double speed = std::max(std::abs(command.linear), std::abs(measured_speed));
    const double reach =
      speed * speed / (2.0 * core_->params().linear_decel) + collision_margin_;
    // One cell per sample, so the footprint outline sweeps every cell.
    const double resolution = costmap->getResolution();
    const int samples = static_cast<int>(std::ceil(reach / resolution));
    const double sign = command.linear > 0.0 ? 1.0 : -1.0;
    for (int i = 1; i <= samples; ++i) {
      const double d = sign * std::min(i * resolution, reach);
      if (inCollision(x + d * std::cos(yaw), y + d * std::sin(yaw), yaw, footprint)) {
        throw nav2_core::NoValidControl(
                name_ + ": obstacle within stopping distance " + std::to_string(reach) + " m");
      }
    }
  } else if (state == State::ALIGN || state == State::FINAL_ALIGN) {
    // The spec only asks for a forward check, but this 1.72 x 0.80 m robot
    // sweeps a circle of about 0.95 m radius when it pivots: its corners can
    // hit things that are nowhere near its front. Sweep the whole remaining
    // rotation before starting it.
    const double rotation = core_->remainingRotation();
    const int samples = static_cast<int>(std::ceil(std::abs(rotation) / rotation_check_step_));
    for (int i = 1; i <= samples; ++i) {
      const double theta = yaw + rotation * i / samples;
      if (inCollision(x, y, theta, footprint)) {
        throw nav2_core::NoValidControl(
                name_ + ": obstacle in the footprint's rotation sweep");
      }
    }
  }
}

bool PointAndTurnController::inCollision(
  double x, double y, double yaw, const std::vector<geometry_msgs::msg::Point> & footprint)
{
  // footprintCostAtPose returns the highest cost on the footprint outline,
  // and LETHAL for any footprint leaving the costmap. NO_INFORMATION (255)
  // sorts above LETHAL (254), so with allow_unknown an unknown cell on one
  // edge can hide a lethal cell on another. Our local costmap doesn't track
  // unknown space, so that can't happen here; set allow_unknown false on a
  // costmap that does.
  const double cost = collision_checker_.footprintCostAtPose(x, y, yaw, footprint);
  if (cost == static_cast<double>(nav2_costmap_2d::NO_INFORMATION)) {
    return !allow_unknown_;
  }
  return cost >= static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
}

void PointAndTurnController::publishDebug()
{
  if (!state_pub_ || !state_pub_->is_activated()) {
    return;
  }
  if (!state_published_ || core_->state() != published_state_) {
    std_msgs::msg::String message;
    message.data = toString(core_->state());
    state_pub_->publish(message);
    published_state_ = core_->state();
    state_published_ = true;
  }
  if (core_->verticesRevision() != published_revision_) {
    nav_msgs::msg::Path path;
    path.header.frame_id = plan_frame_;
    path.header.stamp = clock_->now();
    for (const Point2D & v : core_->vertices()) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = v.x;
      pose.pose.position.y = v.y;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    vertices_pub_->publish(path);
    published_revision_ = core_->verticesRevision();
  }
}

void PointAndTurnController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  std::lock_guard<std::mutex> lock(mutex_);
  double scale = 1.0;
  if (speed_limit != nav2_costmap_2d::NO_SPEED_LIMIT) {
    scale = percentage ? speed_limit / 100.0 : speed_limit / core_->params().max_linear_speed;
  }
  core_->setSpeedScale(scale);
}

void PointAndTurnController::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (core_) {
    core_->reset();
    publishDebug();
  }
}

}  // namespace point_and_turn_controller

PLUGINLIB_EXPORT_CLASS(point_and_turn_controller::PointAndTurnController, nav2_core::Controller)
