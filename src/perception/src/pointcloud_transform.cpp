#include "pointcloud_transform.hpp"

#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr rotate_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, const float x, const float y, const float z) { //
    Eigen::Affine3f affine_matrix = Eigen::Affine3f::Identity();
    affine_matrix.rotate(Eigen::AngleAxisf(x, Eigen::Vector3f::UnitX())); //rotates arg1 radians, arg2 axis
    affine_matrix.rotate(Eigen::AngleAxisf(y, Eigen::Vector3f::UnitY()));
    affine_matrix.rotate(Eigen::AngleAxisf(z, Eigen::Vector3f::UnitZ()));
    // affine_matrix.rotate(Eigen::AngleAxisf(transform.angular.x, Eigen::Vector3f::UnitX()));
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud(*source_cloud, *transformed_cloud, affine_matrix);
    
    return transformed_cloud;
}

PointCloudTransformNode::PointCloudTransformNode(const rclcpp::NodeOptions& options) : rclcpp::Node("pointcloud_tf", options) {
    const auto qos = rclcpp::SystemDefaultsQoS();
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/zed/zed_node/point_cloud/cloud_registered", qos, std::bind(&PointCloudTransformNode::pointcloud_callback, this, std::placeholders::_1));
}



int main (int argc, char** argv) { //copy paste of their template, remove later
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    /* Reminder: how transformation matrices work :

            |-------> This column is the translation
    | 1 0 0 x |  \
    | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
    | 0 0 1 z |  /
    | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

    METHOD #1: Using a Matrix4f
    This is the "manual" method, perfect to understand but error prone !
    */
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

    // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
    float theta = M_PI/4; // The angle of rotation in radians
    transform_1 (0,0) = std::cos (theta);
    transform_1 (0,1) = -sin(theta);
    transform_1 (1,0) = sin (theta);
    transform_1 (1,1) = std::cos (theta);
    //    (row, column)

    // Define a translation of 2.5 meters on the x axis.
    transform_1 (0,3) = 2.5;

    // Print the transformation
    printf ("Method #1: using a Matrix4f\n");
    std::cout << transform_1 << std::endl;

    /*  METHOD #2: Using a Affine3f
    This method is easier and less error prone
    */
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

    // Define a translation of 2.5 meters on the x axis.
    transform_2.translation() << 2.5, 0.0, 0.0;

    // The same rotation matrix as before; theta radians around Z axis
    transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

    // Print the transformation
    printf ("\nMethod #2: using an Affine3f\n");
    std::cout << transform_2.matrix() << std::endl;

    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform_2);

    // Visualization
    printf(  "\nPoint cloud colors :  white  = original point cloud\n"
        "                        red  = transformed point cloud\n");
    pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (source_cloud, 255, 255, 255);
    // We add the point cloud to the viewer and pass the color handler
    viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 230, 20, 20); // Red
    viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

    viewer.addCoordinateSystem (1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
    //viewer.setPosition(800, 400); // Setting visualiser window position

    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce ();
    }

    return 0;
}
