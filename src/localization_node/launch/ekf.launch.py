#!/usr/bin/env python3
"""

This script starts the robot_localization package's EKF node which combines (fuses)
data from wheel odometry and IMU sensors to better estimate the robot's position
and orientation.

"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Generate a launch description for the EKF node.

    This function creates and returns a LaunchDescription object that will start
    the EKF node from the robot_localization package. The node is configured
    using parameters from a YAML file.

    Returns:
        LaunchDescription: A complete launch description for the EKF node
    """
    # Constants for paths to different files and folders
    package_name = 'localization_node'

    # Config file paths
    ekf_config_file_path = 'ekf.yaml'

    # Set the path to different packages
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    # Set the path to config files
    default_ekf_config_path = os.path.join(pkg_share, ekf_config_file_path)

    # Launch configuration variables
    ekf_config_file = LaunchConfiguration('ekf_config_file')

    # Declare the launch arguments
    declare_ekf_config_file_cmd = DeclareLaunchArgument(
        name='ekf_config_file',
        default_value=default_ekf_config_path,
        description='Full path to the EKF configuration YAML file'
    )

    # Specify the actions
    start_ekf_node_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory("localization_node"), 'config/ekf.yaml')],

    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the declarations
    ld.add_action(declare_ekf_config_file_cmd)

    # Add the actions
    ld.add_action(start_ekf_node_cmd)

    return ld