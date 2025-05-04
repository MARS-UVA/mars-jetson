#include "realsense_capture.h"
#include <ctime>
#include <string>
#include "models/obstacle_clustering_tree.h"
#include "gradient_map.h"
#include "local_path_planner_graph.h"
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <filesystem>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#define DECIMATION_KERNEL_SIZE 4

class ObstacleDetectNode : public rclcpp::Node
{
public:
  ObstacleDetectNode()
      : Node("ObstacleDetectNode")
  {
    // publisher_ = this->create_publisher<teleop_msgs::msg::HumanInputState>("human_input_state", 10);
    // subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    //     "webcam_image", 10, std::bind(&NetNode::topic_callback, this, _1));
  }

  void start()
  {
    while (rclcpp::ok())
    {
      vertices = new std::vector<Vertex>();
      std::shared_ptr<Matrices> retMatrices = capture_depth_matrix(this->vertices, DECIMATION_KERNEL_SIZE);
      delete *vertices;
      vertices.reset();
    }
  }

private:
  std::optional<std::vector<Vertex> *> vertices;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleDetectNode>();
  std::thread async_spin([&]()
                         { rclcpp::spin(node); });
  node->start();
  rclcpp::shutdown();
  shm_unlink(SHM_NAME);
  if (async_spin.joinable())
    async_spin.join();
  return 0;
}