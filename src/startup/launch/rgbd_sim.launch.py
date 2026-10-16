"""Bridge and visualize the robot's optional blue and orange RGB-D sensors."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'enable_rviz', default_value='true',
            description='Show the RGB-D or navigation RViz view',
        ),
        DeclareLaunchArgument(
            'rviz_config', default_value=PathJoinSubstitution([
                FindPackageShare('startup'), 'config', 'RGBDRViz.rviz',
            ]),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rgbd_rviz',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=IfCondition(LaunchConfiguration('enable_rviz')),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='rgbd_blue_points_bridge',
            output='screen',
            arguments=[
                '/rgbd_blue/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            ],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                # Native Gazebo cloud coordinates are X-forward, Y-left, Z-up.
                # Apply this label only to points; images need the optical frame.
                'override_frame_id': 'rgbd_blue_link',
            }],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='rgbd_orange_points_bridge',
            output='screen',
            arguments=[
                '/rgbd_orange/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            ],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                # Native Gazebo cloud coordinates are X-forward, Y-left, Z-up.
                # Apply this label only to points; images need the optical frame.
                'override_frame_id': 'rgbd_orange_link',
            }],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='rgbd_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'config_file': PathJoinSubstitution([
                    FindPackageShare('startup'), 'config', 'rgbd_bridge.yaml',
                ]),
            }],
        ),
    ])
