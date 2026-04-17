#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "main.hpp"
#include "server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <teleop_msgs/msg/stick_position.hpp>
#include "teleop_msgs/msg/human_input_state.hpp"
#include <autonomy_msgs/action/autonomous_actions.hpp>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netinet/in.h>
#include <thread>
#include <cstdio>
#include <stdexcept>
#include <csignal>
#include <atomic>
#include <mutex>
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include <iostream>
#include <cerrno>

// PORT FOR RECEIVING
#define PORT 8080

using namespace std::chrono_literals;


// Thread Safety
std::mutex action_mutex;
std::atomic<int> pending_action = 0;
std::atomic<bool> action_update = false;

class udpServer : public rclcpp::Node
{
  public:
    using DigDump = autonomy_msgs::action::AutonomousActions;
    using DigDumpGoalHandle = rclcpp_action::ServerGoalHandle<DigDump>;
    udpServer()
    : Node("udp_server")
    {
      client_ptr_ = rclcpp_action::create_client<DigDump>(this, "digdump");
      goal_handle = nullptr;

      robot_state_toggle_publisher_ = this->create_publisher<std_msgs::msg::UInt8>("robot_state/toggle", 10);
      human_input_state_publisher_ = this->create_publisher<teleop_msgs::msg::HumanInputState>("human_input_state", 10);
      server_active = true;
      current_action_state = 0;

      action_timer_ = this->create_wall_timer(10ms, std::bind(&udpServer::action_timer_callback, this));

      server_thread_ = std::thread([this]() { runServer(); });
    }

  private:
    void action_timer_callback() {
      if(!action_update.load()) return;
      int action = pending_action.load();
      action_update.store(false);

      // Cancel previous goal if one is active
      if (this->goal_handle) {
        cancel_goal();
      }
      // Set new action
      switch (action) {
        // TODO: send action state back to UI after updating (might need to be done elsewhere)
        case 0: {
          current_action_state = 0;
          break;
        }
      
        case DIG_AUTO:
          current_action_state = DIG_AUTO;
          send_goal(DIG_AUTO);
          break;
        
        case DUMP_AUTO:
          current_action_state = DUMP_AUTO;
          send_goal(DUMP_AUTO);
          break;
        
        case ESTOP: {
          current_action_state = ESTOP;
          // Change robot state to ESTOP immediately
          std_msgs::msg::UInt8 msg;
          msg.data = ESTOP;
          robot_state_toggle_publisher_->publish(msg);
          break;
        }
        default:
          RCLCPP_WARN(this->get_logger(), "Received Invalid Action Command");
          break;
      }
    }

    void goal_response_callback(const rclcpp_action::ClientGoalHandle<DigDump>::SharedPtr & goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        this->goal_handle = goal_handle;
      }
    }

    void send_goal(int action_type) {
      if (!this->client_ptr_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available");
        return;
      }

      auto goal_msg = DigDump::Goal();
      goal_msg.index = action_type;

      auto send_goal_options = rclcpp_action::Client<DigDump>::SendGoalOptions();
      // Update this callback to store the handle
      send_goal_options.goal_response_callback = [this](const rclcpp_action::ClientGoalHandle<DigDump>::SharedPtr & handle) {
        if (!handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
          this->goal_handle = handle; // Store the handle for later cancellation
        }
      };

      this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    void cancel_goal() {
      if (!this->goal_handle) {
        RCLCPP_DEBUG(this->get_logger(), "No active goal to cancel");
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Sending cancel request...");
      
      // Use a lambda to nullify the handle once the server acknowledges cancellation
      auto cancel_callback = [this](auto response) {
        (void)response;
        RCLCPP_INFO(this->get_logger(), "Cancel request processed by server");
        this->goal_handle = nullptr; 
      };

      this->client_ptr_->async_cancel_goal(this->goal_handle, cancel_callback);
    }

    int runServer()
    {
      struct sockaddr_in server_addr;
      struct sockaddr_in client_addr;
      memset(&server_addr, '\0', sizeof(server_addr));
      
      int socket_desc = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
      if (socket_desc < 0)
      {
        printf("Error while creating socket\n");
        return -1;
      }
      // Set Port and IP
      server_addr.sin_family = AF_INET;
      server_addr.sin_port = htons(8080);
      // 0.0.0.0 binds to all networks
      server_addr.sin_addr.s_addr = inet_addr("0.0.0.0");

      // Bind to the Port and IP
      int reuse_option = 1;
      if (setsockopt(socket_desc, SOL_SOCKET, SO_REUSEADDR, (const char *)&reuse_option, sizeof(int)) < 0)
      {
        throw std::runtime_error("Error setting socket options");
      }
      struct sockaddr *server_addr_ptr = (struct sockaddr *)&server_addr;

      // Bind the socket
      if (bind(socket_desc, server_addr_ptr, sizeof(server_addr)) < 0)
      {
        std::cout << "Bind error number: " << errno << std::endl;
        throw std::runtime_error("Error binding server socket");
      }

      RCLCPP_INFO(this->get_logger(), "Listening for Incoming Messages");
      // Loop for handling receiving data
      char buffer[1410];
      socklen_t client_len = sizeof(client_addr);
      while (server_active) {
        // Receive Bytes
        ssize_t num_bytes = recvfrom(
          socket_desc, buffer, sizeof(buffer), 0, (struct sockaddr*)&client_addr, &client_len
        );
        // TODO: DELETE THIS ONCE CONFIRMED WORKING
        RCLCPP_DEBUG(this->get_logger(), "Received Message");

        // Can only be negative when there's an error
        if (num_bytes < 0) {
          RCLCPP_WARN(this->get_logger(), "Receiving error");
        }

        // Extract header
        uint8_t* buf = reinterpret_cast<uint8_t*>(buffer);
        udpHeader header = {
          buf[static_cast<int>(HeaderFields::reserved)],
          buf[static_cast<int>(HeaderFields::packetType)],
          buf[static_cast<int>(HeaderFields::packetLength)],
          buf[static_cast<int>(HeaderFields::numPackets)],
          buf[static_cast<int>(HeaderFields::batchPacketCount)],
          buf[static_cast<int>(HeaderFields::crc)]
        };

        // TODO: add crc Verification

        // Get pointer to first byte after header
        char *payload = buffer + sizeof(header);
        // Process Data according to Packet Type
        switch (header.packetType) {
          // Gamepad Inputs
          case static_cast<int>(RecvPacketTypes::HumanInput): {
            // Create msgs to send data
            auto gamepad_msg = teleop_msgs::msg::GamepadState();

            // Save data into the struct for the packet
            GamepadPacket pkt;
            std::memcpy(&pkt, payload, sizeof(GamepadPacket));
            // Save data into the message
            // Joystick Values
            gamepad_msg.left_stick.x = pkt.left_stick_x;
            gamepad_msg.left_stick.y = pkt.left_stick_y;
            gamepad_msg.right_stick.x = pkt.right_stick_x;
            gamepad_msg.right_stick.y = pkt.right_stick_y;
            // Button Values
            gamepad_msg.x_pressed = pkt.x;
            gamepad_msg.y_pressed = pkt.y;
            gamepad_msg.a_pressed = pkt.a;
            gamepad_msg.b_pressed = pkt.b;
            gamepad_msg.lt_pressed = pkt.lt;
            gamepad_msg.rt_pressed = pkt.rt;
            gamepad_msg.lb_pressed = pkt.lb;
            gamepad_msg.rb_pressed = pkt.rb;
            gamepad_msg.dd_pressed = pkt.dd;
            gamepad_msg.du_pressed = pkt.du;
            gamepad_msg.dl_pressed = pkt.dl;
            gamepad_msg.dr_pressed = pkt.dr;
            gamepad_msg.l3_pressed = pkt.l3;
            gamepad_msg.r3_pressed = pkt.r3;
            gamepad_msg.start_pressed = pkt.start;
            gamepad_msg.back_pressed = pkt.back;

            // Put into human_input_msg
            auto human_input_msg = teleop_msgs::msg::HumanInputState();
            human_input_msg.gamepad_state = gamepad_msg;
            switch (current_action_state) {
              case 0: // teleop
                human_input_msg.drive_mode = human_input_msg.DRIVEMODE_TELEOP;
                break;
              case DIG_AUTO:
                human_input_msg.drive_mode = human_input_msg.DRIVEMODE_AUTONOMOUS;
                break;
              case DUMP_AUTO:
                human_input_msg.drive_mode = human_input_msg.DRIVEMODE_AUTONOMOUS;
                break;
              case ESTOP:
              // TODO: I think this can get moved to action commands section
                std_msgs::msg::UInt8 msg;
                msg.data = ESTOP;
                robot_state_toggle_publisher_->publish(msg); //everything else is handled
            }
            // Publish Human Input State msg
            human_input_state_publisher_->publish(human_input_msg);
            break;
          }
          // Action Commands
          case static_cast<int>(RecvPacketTypes::Action): {
            uint8_t robot_action = *payload;
            pending_action.store(robot_action);
            action_update.store(true);
            break;
          }          
          // Config/Settings
          case static_cast<int>(RecvPacketTypes::Config):
            break;
          
          default:
            RCLCPP_WARN(this->get_logger(), "Received Invalid Packet Type");
            break;
        }
      }
      // Close socket when done
      close(socket_desc);
      return 0;
    }

  bool server_active;
  uint8_t current_action_state;

  rclcpp_action::Client<DigDump>::SharedPtr client_ptr_;
  rclcpp_action::ClientGoalHandle<autonomy_msgs::action::AutonomousActions>::SharedPtr goal_handle;
  rclcpp::TimerBase::SharedPtr action_timer_;

  std::thread server_thread_;

  rclcpp::Publisher<teleop_msgs::msg::HumanInputState>::SharedPtr human_input_state_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr robot_state_toggle_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<udpServer>());
  rclcpp::shutdown();
  return 0;
}
