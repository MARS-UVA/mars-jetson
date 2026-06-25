#include <thread>
#include <algorithm>

#include "autonomy_msgs/action/autonomous_actions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/string.hpp"
#include "control_msgs/msg/arm_drum_control.hpp"
#include "control_msgs/msg/arm_control_mode.hpp"
#include "geometry_msgs/msg/twist.hpp"

struct TwistArmDrumControl {
  geometry_msgs::msg::Twist twist;
  control_msgs::msg::ArmDrumControl arm_drum_control;
};

class DigDumpActionServer : public rclcpp::Node
{
  using DigDump = autonomy_msgs::action::AutonomousActions;
  using DigDumpGoalHandle = rclcpp_action::ServerGoalHandle<DigDump>;
  public:
    explicit DigDumpActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    rcl_interfaces::msg::ParameterDescriptor actuator_speed_aerial_descriptor;
    rcl_interfaces::msg::ParameterDescriptor actuator_speed_ground_descriptor;
    rcl_interfaces::msg::ParameterDescriptor raise_speed_descriptor;
    rcl_interfaces::msg::ParameterDescriptor dig_speed_descriptor;
    rcl_interfaces::msg::ParameterDescriptor dump_speed_descriptor;
    rcl_interfaces::msg::ParameterDescriptor drive_speed_descriptor;
    rcl_interfaces::msg::ParameterDescriptor dig_time_descriptor;
    rcl_interfaces::msg::ParameterDescriptor dump_time_descriptor;
    rcl_interfaces::msg::ParameterDescriptor move_time_descriptor;
    rcl_interfaces::msg::ParameterDescriptor actuator_extend_length_aerial_descriptor;
    rcl_interfaces::msg::ParameterDescriptor actuator_extend_length_ground_descriptor;


  private:
    rclcpp_action::Server<DigDump>::SharedPtr action_server_;
    rclcpp::Subscription<serial_msgs::msg::Position>::SharedPtr position_sub_;
    rclcpp::Subscription<control_msgs::msg::ArmControlMode>::SharedPtr arm_control_mode_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<control_msgs::msg::ArmDrumControl>::SharedPtr arm_drum_control_pub_;
    
    //Parameter to track if a goal is currently active. Used to prevent accepting new goals while one is active
    bool goal_active_ = false;
    bool back_arm_control_mode = false;

    // Track active goal
    std::shared_ptr<DigDumpGoalHandle> active_goal_;

    // Cancel command subscriber
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr cancel_sub_;

    // Parameter values
    double actuator_speed_aerial;
    double actuator_speed_ground;
    double dig_speed;
    double dump_speed;
    double drive_speed;

    double dig_time;
    double dump_time;
    double move_time;
    double actuator_extend_length_aerial;
    double actuator_extend_length_ground;

    // tracker for current actuator position as a pointer to update while on another thread
    double current_front_actuator_position;
    double current_back_actuator_position;

    TwistArmDrumControl lower_msg;
    TwistArmDrumControl raise_msg;
    TwistArmDrumControl dig_msg;
    TwistArmDrumControl dump_msg;
    TwistArmDrumControl drive_msg;
    TwistArmDrumControl stop_msg;

    control_msgs::msg::ArmDrumControl lower_arm_drum_msg;
    control_msgs::msg::ArmDrumControl raise_arm_drum_msg;

    void arm_control_mode_callback(const control_msgs::msg::ArmControlMode::SharedPtr msg);

    void actuator_position_callback(const serial_msgs::msg::Position::SharedPtr msg);

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

    void publish_command(const TwistArmDrumControl & command);
};
