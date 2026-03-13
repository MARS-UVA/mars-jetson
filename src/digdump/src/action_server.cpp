#include "digdump/action_server.hpp"

DigDumpActionServer::DigDumpActionServer(const rclcpp::NodeOptions & options) : rclcpp::Node("action_server", options){
  action_server_ = rclcpp_action::create_server<DigDump>(
    this,
    "digdump",
    std::bind(&DigDumpActionServer::handle_goal, this,
              std::placeholders::_1, std::placeholders::_2),
    std::bind(&DigDumpActionServer::handle_cancel, this,
              std::placeholders::_1),
    std::bind(&DigDumpActionServer::handle_accepted, this,
              std::placeholders::_1));

  state_publisher_ = this->create_publisher<std_msgs::msg::UInt8>("robot_state", 1);
  motor_publisher_ = this->create_publisher<teleop_msgs::msg::MotorChanges>("digdump_autonomy", 1);

  teleop_msgs::msg::MotorChanges lower_msg;
  teleop_msgs::msg::MotorChanges raise_msg;
  teleop_msgs::msg::MotorChanges dig_msg;
  teleop_msgs::msg::MotorChanges dump_msg;
  
  const int lower_speed = 112;  //TODO: parameterize
  const int raise_speed = 142;
  const int dig_speed = 157;
  const int dump_speed = 97;

  teleop_msgs::msg::SetMotor msg;

  for (int i=0; i<8; i++) {
    msg.index = i;
    lower_msg.changes.push_back(msg);
    raise_msg.changes.push_back(msg);
    dig_msg.changes.push_back(msg);
    dump_msg.changes.push_back(msg);
  }
  lower_msg.changes[msg.ARM_FRONT_ACTUATOR].velocity = lower_speed;
  raise_msg.changes[msg.ARM_FRONT_ACTUATOR].velocity = raise_speed;
  dig_msg.changes[msg.SPIN_FRONT_DRUM].velocity = dig_speed;
  dump_msg.changes[msg.SPIN_FRONT_DRUM].velocity = dump_speed;

}

rclcpp_action::GoalResponse DigDumpActionServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const DigDump::Goal> goal)
{
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DigDumpActionServer::handle_cancel(
  const std::shared_ptr<DigDumpGoalHandle> goal_handle)
{
  RCLCPP_INFO(rclcpp::get_logger("server"), "Got request to cancel goal");
  auto state = std_msgs::msg::UInt8();
  state.data = 0;
  state_publisher_->publish(state);

  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DigDumpActionServer::execute(
  const std::shared_ptr<DigDumpGoalHandle> goal_handle)
{
  RCLCPP_INFO(rclcpp::get_logger("server"), "Executing goal");
  double arm_movement_time = 5.0; //TODO: parameterize later
  double dig_time = 5.0;
  double dump_time = 5.0;
  double move_time = 5.0;

  rclcpp::Rate loop_rate(10);
  const int action_type = goal_handle->get_goal()->index;
  auto feedback = std::make_shared<DigDump::Feedback>();
  auto result = std::make_shared<DigDump::Result>();
  // Publish feedback
  // goal_handle->publish_feedback(feedback);
  // RCLCPP_INFO(rclcpp::get_logger("server"), "Publish Feedback");
  auto state = std_msgs::msg::UInt8();
  state.data = 1;
  state_publisher_->publish(state);

  switch (action_type) {
    case 0: {
      // demo
      break;
    }
    case 1: {
      // Dig Autonomy
      
      double elapsed_time = 0.0;
      while (elapsed_time < arm_movement_time) {
        motor_publisher_->publish(lower_msg);
        loop_rate.sleep();
        elapsed_time += 0.1;
      }

      elapsed_time = 0.0;
      while (elapsed_time < dig_time) {
        motor_publisher_->publish(dig_msg);
        loop_rate.sleep();
        elapsed_time += 0.1;
      }

      break;
    }
    case 2: {
      // dump

    }
    default: {
      // some kind of error
    }
  }

  state.data = 0;
  state_publisher_->publish(state);
  // Check if goal is done
  if (rclcpp::ok()) {
    goal_handle->succeed(result);
    RCLCPP_INFO(rclcpp::get_logger("server"), "Goal Succeeded");
  }
}

void DigDumpActionServer::handle_accepted(const std::shared_ptr<DigDumpGoalHandle> goal_handle)
{
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&DigDumpActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<DigDumpActionServer> node = std::make_shared<DigDumpActionServer>();

  // Create an action server with three callbacks
  //   'handle_goal' and 'handle_cancel' are called by the Executor (rclcpp::spin)
  //   'execute' is called whenever 'handle_goal' returns by accepting a goal
  //    Calls to 'execute' are made in an available thread from a pool of four.
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
