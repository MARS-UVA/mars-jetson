#include <memory>
#include <string>
#include <vector>

#include <realtime_tools/realtime_buffer.hpp>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include "robot_control_msgs/msg/arm_drum_control.hpp" 

namespace arm_drum_controller {

class ArmDrumController : public controller_interface::ControllerInterface {
public:
    ArmDrumController() = default;

    controller_interface::CallbackReturn on_init() override {
        try {
            // Declare parameter for each individual joint name, expecting exactly 4 joints for this controller
            // auto_declare<std::vector<std::string>>("joint_names", std::vector<std::string>());
            auto_declare<std::vector<std::string>>("front_arm_joints", std::vector<std::string>());
            auto_declare<std::vector<std::string>>("back_arm_joints", std::vector<std::string>());
            auto_declare<std::vector<std::string>>("front_drum_joints", std::vector<std::string>());
            auto_declare<std::vector<std::string>>("back_drum_joints", std::vector<std::string>());
            // Declare parameter for safety command timeout
            auto_declare<double>("command_timeout", 0.2);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_node()->get_logger(), "Exception during controller init: %s", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration command_interface_configuration() const override {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        
        // We request VELOCITY command interfaces for our 4 specific joints
        for (const auto& joints : {front_arm_joint_names_, back_arm_joint_names_, front_drum_joint_names_, back_drum_joint_names_}) {
            for (const auto &joint : joints) {
                config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
            }
        }
        return config;
    }

    controller_interface::InterfaceConfiguration state_interface_configuration() const override {
        // We don't strictly need state interfaces to forward velocities,
        // but we claim individual position/velocity states for completeness or safety tracking
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for (const auto& joints : {front_arm_joint_names_, back_arm_joint_names_, front_drum_joint_names_, back_drum_joint_names_}) {
            for (const auto & joint : joints) {
                config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
                config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
            }
        }
        return config;
    }

    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override {
        front_arm_joint_names_ = get_node()->get_parameter("front_arm_joints").as_string_array();
        back_arm_joint_names_ = get_node()->get_parameter("back_arm_joints").as_string_array();
        front_drum_joint_names_ = get_node()->get_parameter("front_drum_joints").as_string_array();
        back_drum_joint_names_ = get_node()->get_parameter("back_drum_joints").as_string_array();

        timeout_duration_ = rclcpp::Duration::from_seconds(get_node()->get_parameter("command_timeout").as_double());

        // Initialize our incoming command subscription
        subscription_ = get_node()->create_subscription<robot_control_msgs::msg::ArmDrumControl>(
            "~/commands", 10, [this](const std::shared_ptr<robot_control_msgs::msg::ArmDrumControl> msg) {
                CommandData data{msg, get_node()->now()};
                rt_command_ptr_.writeFromNonRT(data);
        });
        
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        // Clear out real-time buffer pointers
        CommandData default_data{std::make_shared<robot_control_msgs::msg::ArmDrumControl>(), get_node()->now()};
        rt_command_ptr_.writeFromNonRT(default_data);
        last_command_time_ = get_node()->now();
        configure_joint_handles();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        // Safe state: Command 0 velocity to everything when turning off the controller
        for (auto& command_interface : command_interfaces_) {
            if (!command_interface.set_value(0.0)) {
                RCLCPP_WARN(get_node()->get_logger(), "Failed to set command interface value for joint %s", command_interface.get_name().c_str());
            }
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& /*period*/) override {
        auto current_command = rt_command_ptr_.readFromRT();

        if (!current_command || !(current_command->msg)) {
            zero_all_commands();
            return controller_interface::return_type::OK;
        }

        // Evaluate age using the exact timestamp the packet hit the subscriber thread
        if (time - current_command->receive_time > timeout_duration_) {
            RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "Watchdog triggered: Command timeout!");
            zero_all_commands();
            return controller_interface::return_type::OK;
        }

        // Update last timestamp if we have a fresh message handled by the non-RT subscriber thread
        last_command_time_ = time;

        // 3. Map semantic message fields explicitly to ros2_control command interfaces.
        for (const auto &front_arm_joint : front_arm_joints_) {
            if (!front_arm_joint.get().set_value(current_command->msg->front_arm_speed)) {
                RCLCPP_WARN(get_node()->get_logger(), "Failed to set command interface value for joint %s", front_arm_joint.get().get_name().c_str());
            }
        }

        for (const auto &back_arm_joint : back_arm_joints_) {
            if (!back_arm_joint.get().set_value(current_command->msg->back_arm_speed)) {
                RCLCPP_WARN(get_node()->get_logger(), "Failed to set command interface value for joint %s", back_arm_joint.get().get_name().c_str());
            }
        }

        for (const auto &front_drum_joint : front_drum_joints_) {
            if (!front_drum_joint.get().set_value(current_command->msg->front_drum_speed)) {
                RCLCPP_WARN(get_node()->get_logger(), "Failed to set command interface value for joint %s", front_drum_joint.get().get_name().c_str());
            }
        }

        for (const auto &back_drum_joint : back_drum_joints_) {
            if (!back_drum_joint.get().set_value(current_command->msg->back_drum_speed)) {
                RCLCPP_WARN(get_node()->get_logger(), "Failed to set command interface value for joint %s", back_drum_joint.get().get_name().c_str());
            }
        }

        return controller_interface::return_type::OK;
    }

    void configure_joint_handle(const std::vector<std::string>& joint_names, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>& joint_handles) {
        for (const auto& joint_name : joint_names) {
            const auto command_handle = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                [&joint_name](const hardware_interface::LoanedCommandInterface& handle) {
                    return handle.get_name() == joint_name + "/" + hardware_interface::HW_IF_VELOCITY;
                });
            
            if (command_handle == command_interfaces_.end()) {
                RCLCPP_ERROR(get_node()->get_logger(), "Command interface for joint %s not found", joint_name.c_str());
                continue;
            }
            joint_handles.push_back(std::ref(*command_handle));
        }
    }

    void zero_all_commands() {
        for (auto& command_interface : command_interfaces_) {
            if (!command_interface.set_value(0.0)) {
                RCLCPP_WARN(get_node()->get_logger(), "Failed to set command interface value for joint %s", command_interface.get_name().c_str());
            }
        }
    }

    void configure_joint_handles() {
        front_arm_joints_.clear();
        back_arm_joints_.clear();
        front_drum_joints_.clear();
        back_drum_joints_.clear();
        
        configure_joint_handle(front_arm_joint_names_, front_arm_joints_);
        configure_joint_handle(back_arm_joint_names_, back_arm_joints_);
        configure_joint_handle(front_drum_joint_names_, front_drum_joints_);
        configure_joint_handle(back_drum_joint_names_, back_drum_joints_);
    }

private:
    struct CommandData {
        std::shared_ptr<robot_control_msgs::msg::ArmDrumControl> msg;
        rclcpp::Time receive_time;
    };

    std::vector<std::string> front_arm_joint_names_;
    std::vector<std::string> back_arm_joint_names_;
    std::vector<std::string> front_drum_joint_names_;
    std::vector<std::string> back_drum_joint_names_;
    
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> front_arm_joints_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> back_arm_joints_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> front_drum_joints_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> back_drum_joints_;

    rclcpp::Subscription<robot_control_msgs::msg::ArmDrumControl>::SharedPtr subscription_;
    
    realtime_tools::RealtimeBuffer<CommandData> rt_command_ptr_;

    rclcpp::Time last_command_time_;
    rclcpp::Duration timeout_duration_{0, 0};

};

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arm_drum_controller::ArmDrumController, controller_interface::ControllerInterface)
