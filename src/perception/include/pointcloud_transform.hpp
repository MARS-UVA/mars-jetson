#pragma once

#include <cmath>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>



class PointCloudTransformNode : public rclcpp::Node
{
public:
  PointCloudTransformNode(const rclcpp::NodeOptions& options);

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
};