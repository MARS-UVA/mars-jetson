#include "client.hpp"

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;


class udpClient : public rclcpp::Node
{
  public:
    udpClient()
    : Node("udp_client")
    {
      create_connection_headers(FEEDBACK_PORT);

      buffer = new unsigned char[FEEDBACK_PACKET_LENGTH]();

      timer_ = this->create_wall_timer(10ms, std::bind(&udpClient::timer_callback, this));

      currentBusSubscription_ = this->create_subscription<serial_msgs::msg::CurrentBusVoltage>(
        "current_bus_voltage", 10, std::bind(&udpClient::current_bus_callback, this, _1)
      );
      temperatureSubscription_ = this->create_subscription<serial_msgs::msg::Temperature>(
        "temperature", 10, std::bind(&udpClient::temperature_callback, this, _1)
      );
      positionSubscription_ = this->create_subscription<serial_msgs::msg::Position>(
        "position", 10, std::bind(&udpClient::position_callback, this, _1)
      );
    }
    ~udpClient() {
      delete[] buffer;
    }

  private:
    void create_connection_headers(int port) {
      const char* CONTROL_STATION_IP_FOR_CLIENT = std::getenv("CONTROL_STATION_IP");
      if (CONTROL_STATION_IP_FOR_CLIENT == nullptr) {
        throw std::runtime_error("CONTROL_STATION_IP not set");
      }

      // Create Socket
      int client_socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
      if (client_socket_fd < 0) {
        throw std::runtime_error("Error while creating socket");
      }

      struct sockaddr_in control_station_addr{};
      socklen_t control_station_struct_len = sizeof(control_station_addr);

      control_station_addr.sin_family = AF_INET;
      control_station_addr.sin_port = htons(port);
      control_station_addr.sin_addr.s_addr = inet_addr(CONTROL_STATION_IP_FOR_CLIENT);

      connection_headers = {client_socket_fd, control_station_addr};
    }

    void client_send(unsigned char *data, size_t data_size) {
      // Create header for message
      udpHeader message_header;
      // TODO: put fields into message header
      // Copy to buffer for send
      std::vector<char> sendBuffer(HEADER_SIZE + data_size);
      memcpy(sendBuffer.data(), &message_header, HEADER_SIZE);
      memcpy(sendBuffer.data() + HEADER_SIZE, data, data_size);
      
      ssize_t transmission_result = sendto(connection_headers.client_socket_fd,
                                            sendBuffer.data(),
                                            data_size + HEADER_SIZE, 0,
                                            (struct sockaddr *)&(connection_headers.control_station_addr),
                                            sizeof(connection_headers.control_station_addr));
                                            
      if (transmission_result < 0) {
        RCLCPP_WARN(this->get_logger(), "Could not send message");
      }
    }

    void current_bus_callback(const serial_msgs::msg::CurrentBusVoltage::SharedPtr msg) {
      RCLCPP_DEBUG(this->get_logger(), "Received current bus voltage feedback packet");

      std::memcpy(&buffer[FeedbackByteIndices::FRONT_LEFT_WHEEL_CURRENT], &msg->front_left_wheel_current, 4);
      std::memcpy(&buffer[FeedbackByteIndices::BACK_LEFT_WHEEL_CURRENT], &msg->back_left_wheel_current, 4);
      std::memcpy(&buffer[FeedbackByteIndices::FRONT_RIGHT_WHEEL_CURRENT], &msg->front_right_wheel_current, 4);
      std::memcpy(&buffer[FeedbackByteIndices::BACK_RIGHT_WHEEL_CURRENT], &msg->back_right_wheel_current, 4);
      std::memcpy(&buffer[FeedbackByteIndices::FRONT_DRUM_CURRENT], &msg->front_drum_current, 4);
      std::memcpy(&buffer[FeedbackByteIndices::BACK_DRUM_CURRENT], &msg->back_drum_current, 4);
      std::memcpy(&buffer[FeedbackByteIndices::FRONT_ACTUATOR_CURRENT], &msg->front_actuator_current, 4);
      std::memcpy(&buffer[FeedbackByteIndices::BACK_ACTUATOR_CURRENT], &msg->back_actuator_current, 4);
      std::memcpy(&buffer[FeedbackByteIndices::MAIN_BATTERY_VOLTAGE], &msg->main_battery_voltage, 4);
      std::memcpy(&buffer[FeedbackByteIndices::AUX_BATTERY_VOLTAGE], &msg->aux_battery_voltage, 4);
    }

    void temperature_callback(const serial_msgs::msg::Temperature::SharedPtr msg) {
      RCLCPP_DEBUG(this->get_logger(), "Received temperature feedback packet");

      std::memcpy(&buffer[FeedbackByteIndices::FRONT_LEFT_WHEEL_TEMPERATURE], &msg->front_left_wheel_temperature, 4);
      std::memcpy(&buffer[FeedbackByteIndices::BACK_LEFT_WHEEL_TEMPERATURE], &msg->back_left_wheel_temperature, 4);
      std::memcpy(&buffer[FeedbackByteIndices::FRONT_RIGHT_WHEEL_TEMPERATURE], &msg->front_right_wheel_temperature, 4);
      std::memcpy(&buffer[FeedbackByteIndices::BACK_RIGHT_WHEEL_TEMPERATURE], &msg->back_right_wheel_temperature, 4);
      std::memcpy(&buffer[FeedbackByteIndices::FRONT_DRUM_TEMPERATURE], &msg->front_drum_temperature, 4);
      std::memcpy(&buffer[FeedbackByteIndices::BACK_DRUM_TEMPERATURE], &msg->back_drum_temperature, 4);
    }

    void position_callback(const serial_msgs::msg::Position::SharedPtr msg) {
      RCLCPP_DEBUG(this->get_logger(), "Received position feedback packet");
      std::memcpy(&buffer[FeedbackByteIndices::FRONT_ACTUATOR_POSITION], &msg->front_actuator_position, 4);
      std::memcpy(&buffer[FeedbackByteIndices::BACK_ACTUATOR_POSITION], &msg->back_actuator_position, 4);
    }

    void robot_state_callback(const std_msgs::msg::UInt8::SharedPtr state) {
      std::memcpy(&buffer[FeedbackByteIndices::ROBOT_STATE], &state->data, 4);
    }

    void timer_callback() {
      client_send(buffer, FEEDBACK_PACKET_LENGTH);
    }

    ConnectionHeaders connection_headers;
    unsigned char* buffer;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr robot_state_subscriber_;
    rclcpp::Subscription<serial_msgs::msg::CurrentBusVoltage>::SharedPtr currentBusSubscription_;
    rclcpp::Subscription<serial_msgs::msg::Temperature>::SharedPtr temperatureSubscription_;
    rclcpp::Subscription<serial_msgs::msg::Position>::SharedPtr positionSubscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<udpClient>());
  rclcpp::shutdown();
  return 0;
}