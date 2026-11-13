#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class CraterDetector final : public rclcpp::Node
{

public:

    CraterDetector();

    

private:

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_sub_cloud_ground;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_sub_cloud_obstacle;

    void cloud_ground_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void cloud_obstacle_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);



};
