#include "crater_detection/crater_detector.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

CraterDetector::CraterDetector()
    : Node { "crater_detector" }
    , m_sub_cloud_ground
    {
        create_subscription<sensor_msgs::msg::PointCloud2>("/rtabmap/cloud_ground", 10, std::bind(&CraterDetector::cloud_ground_callback, this, std::placeholders::_1))
    }
    , m_sub_cloud_obstacles
    {
        create_subscription<sensor_msgs::msg::PointCloud2>("/rtabmap/cloud_obstacles", 10, std::bind(&CraterDetector::cloud_obstacles_callback, this, std::placeholders::_1))
    }
{
    RCLCPP_INFO(get_logger(), "starting to do stuff...");
}

void CraterDetector::cloud_ground_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "received GROUND point cloud: %d points (w %d, h %d)", msg->width * msg->height, msg->width, msg->height);

    // ros2 topic echo /rtabmap/cloud_ground
    // height = 1 probably
    // is dense = true
    // field names = { "x", "y", "z", "rgb" }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud { new pcl::PointCloud<pcl::PointXYZ> };
    pcl::fromROSMsg(*msg, *cloud);

    // for (const pcl::PointXYZ& point : cloud->points)
    // {
    //     const auto& [ x, y, z, _ ] { point.data }; // floats
    // }

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled { new pcl::PointCloud<pcl::PointXYZ> };
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(0.02f, 0.02f, 0.02f); // in meters
    voxel.filter(*downsampled);
    
    RCLCPP_INFO(get_logger(), "cloud->size() = %ld -> downsampled->size() = %ld", cloud->size(), downsampled->size());

    pcl::ModelCoefficients::Ptr coefficients { new pcl::ModelCoefficients };
    pcl::PointIndices::Ptr inliers { new pcl::PointIndices };

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.02); // in meters
    seg.setMaxIterations(1000);
    seg.setInputCloud(downsampled);
    seg.segment(*inliers, *coefficients);

    float a { coefficients->values[0] };
    float b { coefficients->values[1] };
    float c { coefficients->values[2] };
    float d { coefficients->values[3] };

    // change equation from ax + by + cz + d = 0 to z = a'x + b'y + d'
    a /= -c;
    b /= -c;
    d /= -c;
    c = 1.f;

    RCLCPP_INFO(get_logger(), "plane: z = %.6fx + %.6fy + %.6f", a, b, d);

    const float inlier_ratio { (float)inliers->indices.size() / (float)downsampled->size() };

    RCLCPP_INFO(get_logger(), "inlier ratio: %.3f", inlier_ratio);
}

void CraterDetector::cloud_obstacles_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "received OBSTACLES point cloud: %d points (w %d, h %d)", msg->width * msg->height, msg->width, msg->height);

    // ros2 topic echo /rtabmap/cloud_obstacles
    // height = 1 probably
    // is dense = true
    // field names = { "x", "y", "z", "rgb" }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud { new pcl::PointCloud<pcl::PointXYZ> };
    pcl::fromROSMsg(*msg, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled { new pcl::PointCloud<pcl::PointXYZ> };
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(0.02f, 0.02f, 0.02f); // in meters
    voxel.filter(*downsampled);
    
    RCLCPP_INFO(get_logger(), "cloud->size() = %ld -> downsampled->size() = %ld", cloud->size(), downsampled->size());
}
