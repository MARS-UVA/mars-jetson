#include <std_msgs/msg/float32.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>

class SerialNode : public rclcpp::Node {
    public:
        SerialNode(const rclcpp::NodeOptions& options);
        
        void readFromNucleo();
        void updateCurrents();
};