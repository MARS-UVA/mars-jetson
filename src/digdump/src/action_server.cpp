#include "digdump/action_server.hpp"

rclcpp_action::GoalResponse handle_goal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const DigDump::Goal> goal)
{
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse handle_cancel(
  const std::shared_ptr<DigDumpGoalHandle> goal_handle)
{
  RCLCPP_INFO(rclcpp::get_logger("server"), "Got request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void execute(
  const std::shared_ptr<DigDumpGoalHandle> goal_handle)
{
  RCLCPP_INFO(rclcpp::get_logger("server"), "Executing goal");
  rclcpp::Rate loop_rate(1);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<DigDump::Feedback>();
  auto result = std::make_shared<DigDump::Result>();
    // Publish feedback
    // goal_handle->publish_feedback(feedback);
    // RCLCPP_INFO(rclcpp::get_logger("server"), "Publish Feedback");

    loop_rate.sleep();

  // Check if goal is done
  if (rclcpp::ok()) {
    goal_handle->succeed(result);
    RCLCPP_INFO(rclcpp::get_logger("server"), "Goal Succeeded");
  }
}

void handle_accepted(const std::shared_ptr<DigDumpGoalHandle> goal_handle)
{
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{execute, goal_handle}.detach();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("minimal_action_server");

  // Create an action server with three callbacks
  //   'handle_goal' and 'handle_cancel' are called by the Executor (rclcpp::spin)
  //   'execute' is called whenever 'handle_goal' returns by accepting a goal
  //    Calls to 'execute' are made in an available thread from a pool of four.
  auto action_server = rclcpp_action::create_server<DigDump>(
    node,
    "digdump_server",
    handle_goal,
    handle_cancel,
    handle_accepted);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}