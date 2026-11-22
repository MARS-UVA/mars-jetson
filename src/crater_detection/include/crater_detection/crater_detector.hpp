#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// everything is in meters

class CraterDetector final : public rclcpp::Node
{

public:

    CraterDetector();

    inline static const float depth_threshold { -0.05f }; // in meters

private:

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_sub_cloud_ground;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_sub_cloud_obstacles;

    // ground plane coefficients (z = ax + by + c)
    float m_ground_coeff_a;

    float m_ground_coeff_b;

    float m_ground_coeff_c;

    // inlier ratio = inliers->indices.size() / downsampled->size(), should be 0..1
    float m_inlier_ratio;

    // vector of individual clusters
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> m_clusters;

    // downsample (size in meters)
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float size = 0.01f);

    // subscriber callbacks
    void cloud_ground_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void cloud_obstacles_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // get z coordinate of ground plane given an x,y coordinate
    float ground_z_at(float x, float y) const;
    
};
