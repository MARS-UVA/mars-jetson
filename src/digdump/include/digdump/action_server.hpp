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

    rcl_interfaces::msg::ParameterDescriptor lower_speed_descriptor;
    rcl_interfaces::msg::ParameterDescriptor raise_speed_descriptor;
    rcl_interfaces::msg::ParameterDescriptor dig_speed_descriptor;
    rcl_interfaces::msg::ParameterDescriptor dump_speed_descriptor;
    rcl_interfaces::msg::ParameterDescriptor drive_speed_descriptor;
    rcl_interfaces::msg::ParameterDescriptor dig_arm_movement_time_descriptor;
    rcl_interfaces::msg::ParameterDescriptor dig_time_descriptor;
    rcl_interfaces::msg::ParameterDescriptor dump_time_descriptor;
    rcl_interfaces::msg::ParameterDescriptor move_time_descriptor;


  private:
    rclcpp_action::Server<DigDump>::SharedPtr action_server_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_publisher_;
    rclcpp::Publisher<teleop_msgs::msg::MotorChanges>::SharedPtr motor_publisher_;
    
    //Parameter to track if a goal is currently active. Used to prevent accepting new goals while one is active
    bool goal_active_ = false;

    // Track active goal
    std::shared_ptr<DigDumpGoalHandle> active_goal_;

    // Cancel command subscriber
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr cancel_sub_;

    //Parameter to track if a goal is currently active. Used to prevent accepting new goals while one is active
    bool goal_active_ = false;

    // Parameter values
    int actuator_speed;
    int dig_speed;
    int dump_speed;
    int drive_speed;

    double dig_arm_movement_time;
    double dig_time;
    double dump_time;
    double move_time;

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

    void cancel_current_goal(
      std_msgs::msg::UInt8 & state,
      const std::shared_ptr<DigDumpGoalHandle> goal_handle);

    void execute(
      const std::shared_ptr<DigDumpGoalHandle> goal_handle);

    void handle_accepted(const std::shared_ptr<DigDumpGoalHandle> goal_handle);
};