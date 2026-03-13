#include "digdump/action_server.hpp"
#include <std_msgs/msg/detail/u_int8__struct.hpp>

/*class DigDumpActionServer : public rclcpp::Node
{
public:

  explicit DigDumpActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("digdump_action_server") {

  }

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

    //  might have to change to this, but don't know
    // auto execute_in_thread = [this, goal_handle](){return this->execute(goal_handle);};
    // std::thread{execute_in_thread}.detach();
  }

}*/

// duplicates outside of class (not sure whether we want the server class
// (the python file uses it, but don't know if there was a reason for not implementing it in the boilerplate code))
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
      int drum_speed = 112; 
      int actuator_speed = 150;
      

      

      break;
    }
    case 2: {
      // dump

    }
    default: {
      // some kind of error
    }

    state.data = 0;
    state_publisher_->publish(state);
  }


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


// [all wheels' velocity, both arms' velocity, both drums' velocity]
// interface that publishes to a topic with the above info

// void send_msg(double all_velocity, double front_arm_velocity, double back_arm_velocity, double front_drum_velocity, double back_drum_velocity) {
//   pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("motor", 10);
//   std_msgs::msg::Float64MultiArray msg;
//   msg.data = {all_velocity, front_arm_velocity, back_arm_velocity, front_drum_velocity, back_drum_velocity};
// }

// dummy function, will be replaced with actual arm movement code
// void move_arms(int velocity, boolean front_arm, boolean back_arm, boolean up) {

// }

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
