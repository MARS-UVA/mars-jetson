#ifndef NAV2_LAZY_THETA_STAR_PLANNER__LAZY_THETA_STAR_PLANNER_HPP_
#define NAV2_LAZY_THETA_STAR_PLANNER__LAZY_THETA_STAR_PLANNER_HPP_

#include <memory>
#include <string>

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_lazy_theta_star_planner
{

class LazyThetaStarPlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;
  double turn_penalty_weight_{0.6};
  double interpolation_step_{0.05};
  double smoothing_radius_{0.6};
  bool allow_unknown_{true};
};

}  // namespace nav2_lazy_theta_star_planner

#endif  // NAV2_LAZY_THETA_STAR_PLANNER__LAZY_THETA_STAR_PLANNER_HPP_
