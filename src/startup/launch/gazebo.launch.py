# Copyright 2024 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable

from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def launch_rtabmap(context):
    if not IfCondition(LaunchConfiguration('enable_rtabmap')).evaluate(context):
        return []
    if not IfCondition(LaunchConfiguration('enable_d435i')).evaluate(context):
        raise RuntimeError('enable_rtabmap requires enable_d435i:=true')

    # Resolve the optional package only when RTAB-Map is requested.
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('rtabmap_launch'), 'launch', 'rtabmap.launch.py',
        ])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'frame_id': 'frame_assembly',
            'visual_odometry': 'true',
            'publish_tf_odom': 'true',
            'vo_frame_id': 'odom',
            'odom_topic': '/rtabmap/visual_odom',
            'rgb_topic': '/d435i/color/image_raw',
            'depth_topic': '/d435i/depth/image_raw',
            'camera_info_topic': '/d435i/camera_info',
            'approx_sync': 'true',
            'qos': '2',
            'rtabmap_viz': 'true',
            'rviz': 'false',
            'database_path': LaunchConfiguration('rtabmap_database_path'),
        }.items(),
    )]


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # Nodes important for running the simulation and controlling the robot
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'Bruno', '-allow_renaming', 'true',
                    '-x', '1.09',
                    '-y', '-1.780',
                    '-z', '0.12',
                    '-Y', '1.5708',],
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/unilidar/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/unilidar/cloud/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
        ],
        remappings=[
            ('/unilidar/cloud/points', '/unilidar/cloud')
        ],
        output='screen'
    )

    camera_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/front_camera/image_raw', '/rear_camera/image_raw'],
        parameters=[{'qos': 'sensor_data', 'use_sim_time': use_sim_time}],
        output='screen',
    )

    mock_actuator_feedback = Node(
        package='startup',
        executable='actuator_position_feedback',
        name='actuator_position_feedback',
        parameters=[PathJoinSubstitution([
            FindPackageShare('startup'), 'config',
            'actuator_feedback.yaml']), {'use_sim_time': use_sim_time}],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('startup'), 'config', 'robot.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    ld = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'enable_rtabmap', default_value='false',
            description='Run RGB-D visual odometry and RTAB-Map',
        ),
        DeclareLaunchArgument(
            'rtabmap_database_path', default_value='/tmp/d435i_visual_test.db',
            description='RTAB-Map database to save or resume',
        ),
        DeclareLaunchArgument(
            'enable_d435i', default_value=LaunchConfiguration('enable_rtabmap'),
            description='Bridge the optional front RGB-D camera (also enable it in robot_description)',
        ),
        OpaqueFunction(function=launch_rtabmap),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('startup'), 'launch', 'd435i_sim.launch.py',
            ])),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=IfCondition(LaunchConfiguration('enable_d435i')),
        ),
        mock_actuator_feedback,
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[PathJoinSubstitution([FindPackageShare('startup'), 'models'])]
        ),
        bridge,
        camera_bridge,
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [f' -r -v 1 ', 
                    PathJoinSubstitution([
                        FindPackageShare('startup'),
                        'world',
                        'arena_nasa.world'
                    ])
            ])]
        ),
        gz_spawn_entity,
    ])
    return ld
