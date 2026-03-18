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

  auto declare_with_desc = [this](const std::string &name, auto default_val, const std::string &description){
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = description;
    return this->declare_parameter(name, default_val, desc);
  };

  // Speed is from 0-127 where 0 is stopped and 127 is maximum speed
  actuator_speed = declare_with_desc("actuator_speed", 1, "The speed at which the arms lower/raise during dig/dump autonomy routine from 0-127");
  dig_speed = declare_with_desc("dig_speed", 1, "The speed at which the front drums spin during the dig autonomy routine from 0-127");
  dump_speed = declare_with_desc("dump_speed", 1, "The speed at which the front drums spin during the dump autonomy routine from 0-127");
  drive_speed = declare_with_desc("drive_speed", 1, "The speed at which the robot drives during the dump autonomy routine from 0-127");

  // Time parameters are in seconds
  dig_arm_movement_time = declare_with_desc("dig_arm_movement_time", 5.0, "The amount of time the front arms spend moving during the dig autonomy routine");
  dump_arm_movement_time = declare_with_desc("dump_arm_movement_time", 5.0, "The amount of time the front arms spend moving during the dump autonomy routine");
  dig_time = declare_with_desc("dig_time", 5.0, "The amount of time the front drums spend spinning during the dig autonomy routine");
  dump_time = declare_with_desc("dump_time", 5.0, "The amount of time the front drums spend spinning during the dump autonomy routine");
  move_time = declare_with_desc("move_time", 5.0, "The amount of time the robot spends driving during the dump autonomy routine");

  teleop_msgs::msg::SetMotor msg;

  for (int i=0; i<8; i++) {
    msg.index = i;
    lower_msg.changes.push_back(msg);
    raise_msg.changes.push_back(msg);
    dig_msg.changes.push_back(msg);
    dump_msg.changes.push_back(msg);
    drive_msg.changes.push_back(msg);
  }
  lower_msg.changes[msg.ARM_FRONT_ACTUATOR].velocity = actuator_speed*-1;
  raise_msg.changes[msg.ARM_FRONT_ACTUATOR].velocity = actuator_speed;
  dig_msg.changes[msg.SPIN_FRONT_DRUM].velocity = dig_speed + 127;
  dump_msg.changes[msg.SPIN_FRONT_DRUM].velocity = dump_speed + 127;
  drive_msg.changes[msg.DRIVE_FRONT_LEFT_WHEEL].velocity = drive_speed + 127;
  drive_msg.changes[msg.DRIVE_FRONT_RIGHT_WHEEL].velocity = drive_speed + 127;
  drive_msg.changes[msg.DRIVE_BACK_LEFT_WHEEL].velocity = drive_speed + 127;
  drive_msg.changes[msg.DRIVE_BACK_RIGHT_WHEEL].velocity = drive_speed + 127;

  stop_msg.changes[msg.FRONT_LEFT_DRIVE_MOTOR].velocity = 127;
  stop_msg.changes[msg.FRONT_RIGHT_DRIVE_MOTOR].velocity = 127;
  stop_msg.changes[msg.BACK_LEFT_DRIVE_MOTOR].velocity = 127;
  stop_msg.changes[msg.BACK_RIGHT_DRIVE_MOTOR].velocity = 127;
  stop_msg.changes[msg.SPIN_FRONT_DRUM].velocity = 127;
  stop_msg.changes[msg.ARM_FRONT_ACTUATOR].velocity = 127;

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
  double dig_arm_movement_time = this->get_parameter("dig_arm_movement_time").as_double();
  double dump_arm_movement_time = this->get_parameter("dump_arm_movement_time").as_double();
  double dig_time = this->get_parameter("dig_time").as_double();
  double dump_time = this->get_parameter("dump_time").as_double();
  double move_time = this->get_parameter("move_time").as_double();


  std::cout<<lower_msg.changes.size()<<std::endl;
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
      while (elapsed_time < dig_arm_movement_time) {
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

      // Drive forward
      double elapsed_time = 0.0;
      while (elapsed_time < move_time) {
        motor_publisher_->publish(drive_msg);
        loop_rate.sleep();
        elapsed_time += 0.1;
      }

      // Lower arms slightly
      elapsed_time = 0.0;
      while (elapsed_time < dump_arm_movement_time) {
        motor_publisher_->publish(lower_msg);
        loop_rate.sleep();
        elapsed_time += 0.1;
      }

      // Spin drums in reverse
      elapsed_time = 0.0;
      while (elapsed_time < dump_time) {
        motor_publisher_->publish(dump_msg);
        loop_rate.sleep();
        elapsed_time += 0.1;
      }

      break;
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
