#include <thread>
#include "autonomy_msgs/action/autonomous_actions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/string.hpp"
#include "teleop_msgs/msg/add_motor.hpp"
#include "teleop_msgs/msg/set_motor.hpp"
#include "teleop_msgs/msg/motor_changes.hpp"

class DigDumpActionServer : public rclcpp::Node
{
  using DigDump = autonomy_msgs::action::AutonomousActions;
  using DigDumpGoalHandle = rclcpp_action::ServerGoalHandle<DigDump>;
  public:
    explicit DigDumpActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    auto lower_speed;
    auto raise_speed;
    auto dig_speed;
    auto dump_speed;
    auto drive_speed;
    auto dig_arm_movement_time;
    auto dump_arm_movement_time;
    auto dig_time;
    auto dump_time;
    auto move_time;
    



  private:
    rclcpp_action::Server<DigDump>::SharedPtr action_server_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_publisher_;
    rclcpp::Publisher<teleop_msgs::msg::MotorChanges>::SharedPtr motor_publisher_;

    auto lower_speed_descriptor;
    auto raise_speed_descriptor;
    auto dig_speed_descriptor;
    auto dump_speed_descriptor;
    auto drive_speed_descriptor;
    auto dig_arm_movement_time_descriptor;
    auto dump_arm_movement_time_descriptor;
    auto dig_time_descriptor;
    auto dump_time_descriptor;
    auto move_time_descriptor;

    teleop_msgs::msg::MotorChanges lower_msg;
    teleop_msgs::msg::MotorChanges raise_msg;
    teleop_msgs::msg::MotorChanges dig_msg;
    teleop_msgs::msg::MotorChanges dump_msg;
    teleop_msgs::msg::MotorChanges drive_msg;
    teleop_msgs::msg::MotorChanges stop_msg;
    rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const DigDump::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<DigDumpGoalHandle> goal_handle);

    void execute(
      const std::shared_ptr<DigDumpGoalHandle> goal_handle);

    void handle_accepted(const std::shared_ptr<DigDumpGoalHandle> goal_handle);

    void cancel_current_goal(auto state, const std::shared_ptr<DigDumpGoalHandle> goal_handle);
};