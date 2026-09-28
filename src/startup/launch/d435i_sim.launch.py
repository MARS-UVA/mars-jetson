"""Bridge and visualize the robot's optional front RGB-D sensor."""

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
            name='d435i_rviz',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('startup'), 'config', 'RGBDRViz.rviz',
            ])],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='d435i_points_bridge',
            output='screen',
            arguments=[
                '/d435i/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            ],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                # Native Gazebo cloud coordinates are X-forward, Y-left, Z-up.
                # Apply this label only to points; images need the optical frame.
                'override_frame_id': 'd435i_link',
            }],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='d435i_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'config_file': PathJoinSubstitution([
                    FindPackageShare('startup'), 'config', 'd435i_bridge.yaml',
                ]),
            }],
        ),
    ])
