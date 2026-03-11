#include "digdump/action_server.hpp"

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

std::shared_ptr<DigDumpGoalHandle> current_action;
rclcpp_action::GoalResponse handle_goal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const DigDump::Goal> goal)
{
  (void)uuid;
  if (current_action) {
    return rclcpp_action::GoalResponse::REJECT;
  }
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

  switch (goal) {
    case 0: {
      // demo

      break;
    }
    case 1: {
      // dig
      msg = UInt8()
      msg.data = 0
      //self.teleop_state_publisher.publish(msg)
      // Dig Autonomy
      int drum_speed = 200;
      int actuator_speed = 200;
      int wheel_speed = 200;
      //drum_speed = int(self.get_parameter(self.dig_spin_drums_speed_param_descriptor.name).value * 127 + 127)
      //actuator_speed = int(self.get_parameter(self.dig_drum_arm_magnitude_param_descriptor.name).value * 127 + 127)
      // wheel_speed = WheelSpeeds(
      //     self.get_parameter(self.dig_wheel_speed_param_descriptor.name).value,
      //     self.get_parameter(self.dig_wheel_speed_param_descriptor.name).value,
      // )
      // drum_lowering_delay = self.get_parameter(self.drum_dig_lowering_time_param_descriptor.name).value
      // drum_raising_delay = self.get_parameter(self.drum_dig_raising_time_param_descriptor.name).value
      // dig_time = self.get_parameter(self.dig_time_param_descriptor.name).value
      // # Stop all motors currently moving on the robot
      // self.serial_publisher.publish(motor_queries.stop_motors())
      // # Create msg to send initial state
      // msg = MotorChanges(changes=[], adds=[])
      // # Set Drums to start digging
      // msg.changes.append(SetMotor(index=SetMotor.SPIN_FRONT_DRUM, velocity=drum_speed))
      // msg.changes.append(SetMotor(index=SetMotor.SPIN_BACK_DRUM, velocity=get_negative_command(drum_speed)))
      // # Start Lowering of Drums
      // motor_queries.move_arms(actuator_speed, True, True, False, msg)
      // motor_queries.move_arms(actuator_speed, True, False, False, msg)
      // # Send initial msg to serial node
      // self.serial_publisher.publish(msg)
      // # Sleep while drums lower
      // time.sleep(drum_lowering_delay)
      // # Start Driving Forward (Shouldn't do this but we're gonna fix this later) (probably try `msg = MotorChanges(changes=[], adds=[])`)
      // #msg = motor_queries.wheel_speed_to_motor_queries(wheel_speed)
      // MotorChanges(changes=[], adds=[])
      // # Stop drum lowering
      // motor_queries.move_arms(127, True, True, False, msg)
      // motor_queries.move_arms(127, True, False, False, msg)
      // # Send message to drive and dig
      // self.serial_publisher.publish(msg)
      // # Sleep while digging and driving forward
      // time.sleep(dig_time)
      // # Stop Digging
      // self.serial_publisher.publish(motor_queries.stop_motors())
      // # raise drums
      // msg = MotorChanges(changes=[], adds=[])
      // motor_queries.move_arms(actuator_speed, True, True, True, msg)
      // motor_queries.move_arms(actuator_speed, True, False, True, msg)
      // self.serial_publisher.publish(msg)
      // # Sleep while drums raise
      // time.sleep(drum_raising_delay)
      // # Stop drum raising
      // msg = MotorChanges(changes=[], adds=[])
      // motor_queries.move_arms(127, True, True, False, msg)
      // self.serial_publisher.publish(msg)

      // # debug
      // time.sleep(1)
      // self.serial_publisher.publish(motor_queries.stop_motors())

      break;
    }
    case 2: {
      // dump

    }
    default: {
      // some kind of error
    }
  }


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


// [all wheels' velocity, both arms' velocity, both drums' velocity]
// interface that publishes to a topic with the above info

void send_msg(double all_velocity, double front_arm_velocity, double back_arm_velocity, double front_drum_velocity, double back_drum_velocity) {
  pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("motor", 10);
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {all_velocity, front_arm_velocity, back_arm_velocity, front_drum_velocity, back_drum_velocity};


}

// dummy function, will be replaced with actual arm movement code
void move_arms(int velocity, boolean front_arm, boolean back_arm, boolean up) {

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
    "digdump",
    handle_goal,
    handle_cancel,
    handle_accepted);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
