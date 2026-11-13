#include "crater_detection/crater_detector.hpp"

CraterDetector::CraterDetector()
    : Node { "crater_detector" }
    , m_sub_cloud_ground
    {
        create_subscription<sensor_msgs::msg::PointCloud2>("/rtabmap/cloud_ground", 10, std::bind(&CraterDetector::cloud_ground_callback, this, std::placeholders::_1))
    }
    , m_sub_cloud_obstacle
    {
        create_subscription<sensor_msgs::msg::PointCloud2>("/rtabmap/cloud_obstacle", 10, std::bind(&CraterDetector::cloud_obstacle_callback, this, std::placeholders::_1))
    }
{
    RCLCPP_INFO(get_logger(), "starting to do stuff...");
}

void CraterDetector::cloud_ground_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "received ground point cloud: %d points (w %d, h %d)", msg->width * msg->height, msg->width, msg->height);
    
}

void CraterDetector::cloud_obstacle_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "received obstacle point cloud: %d points (w %d, h %d)", msg->width * msg->height, msg->width, msg->height);
    
}
