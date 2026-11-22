#include "crater_detection/crater_detector.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/angles.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>

// ros2 topic echo /rtabmap/cloud_ground
// height = 1 probably
// is dense = true
// field names = { "x", "y", "z", "rgb" }

// ros2 topic echo /rtabmap/cloud_obstacles
// height = 1 probably
// is dense = true
// field names = { "x", "y", "z", "rgb" }

CraterDetector::CraterDetector()
    : Node { "crater_detector" }
    , m_sub_cloud_ground
    {
        create_subscription<sensor_msgs::msg::PointCloud2>("/rtabmap/local_grid_ground", 10, std::bind(&CraterDetector::cloud_ground_callback, this, std::placeholders::_1))
    }
    , m_sub_cloud_obstacles
    {
        create_subscription<sensor_msgs::msg::PointCloud2>("/rtabmap/local_grid_obstacle", 10, std::bind(&CraterDetector::cloud_obstacles_callback, this, std::placeholders::_1))
    }
    , m_ground_coeff_a { 0.f }
    , m_ground_coeff_b { 0.f }
    , m_ground_coeff_c { 0.f }
    , m_inlier_ratio { 0.f }
    , m_clusters { }
{
    RCLCPP_INFO(get_logger(), "starting to do stuff...");
}

void CraterDetector::cloud_ground_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "received GROUND point cloud: %d points (w %d, h %d)", msg->width * msg->height, msg->width, msg->height);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud { new pcl::PointCloud<pcl::PointXYZ> };
    pcl::fromROSMsg(*msg, *cloud);
    cloud = downsample(cloud);
    // note: downsampling (size <= 0.01f) does nothing if with rtabmap_args = (... '--Grid/CellSize 0.01 --cloud_output_voxelized true')

    pcl::ModelCoefficients::Ptr coefficients { new pcl::ModelCoefficients };
    pcl::PointIndices::Ptr inliers { new pcl::PointIndices };

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setAxis(Eigen::Vector3f(0.f, 0.f, 1.f));
    seg.setEpsAngle(pcl::deg2rad(25.f));
    seg.setDistanceThreshold(0.08); // in meters
    seg.setMaxIterations(1000);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    float ir { (float)inliers->indices.size() / (float)cloud->size() };
    if (ir < 0.3f)
    {
        RCLCPP_INFO(get_logger(), "cloud ground rejected; inlier ratio too low (%.3f)", ir);
        return;
    }
    m_inlier_ratio = ir;

    // coefficients contains coefficients for the plane ax + by + cz + d = 0
    // where a, b, c, d correspond to coefficients->values[0], [1], [2], [3]
    // we use the plane in this format z = a'x + b'y + c'
    // where a', b', c' correspond to m_ground_coeff_a, _b, _c
    const float z_coeff { coefficients->values[2] };
    m_ground_coeff_a = coefficients->values[0] / -z_coeff;
    m_ground_coeff_b = coefficients->values[1] / -z_coeff;
    m_ground_coeff_c = coefficients->values[3] / -z_coeff;

    RCLCPP_INFO(get_logger(), "plane: z = %.5fx + %.5fy + %.5f", m_ground_coeff_a, m_ground_coeff_b, m_ground_coeff_c);
    RCLCPP_INFO(get_logger(), "inlier ratio: %.3f", m_inlier_ratio);

    // filter for points below ground level
    for (size_t i { 0 }; i < cloud->size(); i++)
    {
        const pcl::PointXYZ& point { cloud->at(i) };
        const auto& [ x, y, z, _ ] { point.data }; // floats
        if (z - ground_z_at(x, y) > depth_threshold)
        {
            cloud->erase(cloud->begin() + i);
            i--;
        }
    }

    RCLCPP_INFO(get_logger(), "depressed points count = %ld", cloud->size());

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree { new pcl::search::KdTree<pcl::PointXYZ> };
    std::vector<pcl::PointIndices> all_cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> extraction;

    tree->setInputCloud(cloud);
    extraction.setClusterTolerance(0.1); // in meters; how close neighboring points must be to be in same cluster
    extraction.setMinClusterSize(10);
    extraction.setSearchMethod(tree);
    extraction.setInputCloud(cloud);
    extraction.extract(all_cluster_indices);

    RCLCPP_INFO(get_logger(), "found %ld object clusters", all_cluster_indices.size());

    for (pcl::PointIndices& cluster_indices : all_cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster { new pcl::PointCloud<pcl::PointXYZ> };
        for (const int index : cluster_indices.indices)
        {
            cluster->points.push_back(cloud->points[index]);
        }
        cluster->width = cluster->size();
        cluster->height = 1;
        m_clusters.emplace_back(cluster);
        RCLCPP_INFO(get_logger(), "a cluster has %ld points", cluster->size());
    }
}

// this might be useless
void CraterDetector::cloud_obstacles_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "received OBSTACLES point cloud %d points (w %d, h %d)", msg->width * msg->height, msg->width, msg->height);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud { new pcl::PointCloud<pcl::PointXYZ> };
    pcl::fromROSMsg(*msg, *cloud);
    cloud = downsample(cloud);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CraterDetector::downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float size)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled { new pcl::PointCloud<pcl::PointXYZ> };
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(size, size, size); // in meters
    voxel.filter(*downsampled);
    RCLCPP_INFO(get_logger(), "original cloud size -> downsampled cloud size = %ld -> %ld", cloud->size(), downsampled->size());
    return downsampled;
}

float CraterDetector::ground_z_at(float x, float y) const
{
    return (m_ground_coeff_a * x) + (m_ground_coeff_b * y) + m_ground_coeff_c;
}
