#include <thread>
#include "autonomy_msgs/action/autonomous_actions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using DigDump = autonomy_msgs::action::AutonomousActions;
using DigDumpGoalHandle = rclcpp_action::ServerGoalHandle<DigDump>;

rclcpp_action::GoalResponse handle_goal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const DigDump::Goal> goal);

rclcpp_action::CancelResponse handle_cancel(
  const std::shared_ptr<DigDumpGoalHandle> goal_handle);

void execute(
  const std::shared_ptr<DigDumpGoalHandle> goal_handle);

void handle_accepted(const std::shared_ptr<DigDumpGoalHandle> goal_handle);