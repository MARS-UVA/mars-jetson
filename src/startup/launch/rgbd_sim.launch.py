"""Bridge and visualize the robot's optional blue and orange RGB-D sensors."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rgbd_rviz',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('startup'), 'config', 'RGBDRViz.rviz',
            ])],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
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
