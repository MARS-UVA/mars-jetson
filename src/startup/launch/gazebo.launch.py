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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable

from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node, SetParameter, SetRemap
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def launch_rgbd_sync(context):
    if not IfCondition(LaunchConfiguration('RTABMap_sync')).evaluate(context):
        return []
    if not IfCondition(LaunchConfiguration('enable_rgbd')).evaluate(context):
        raise RuntimeError('RTABMap_sync requires enable_rgbd:=true')

    nodes = []
    for camera in ('blue', 'orange'):
        prefix = f'/rgbd_{camera}'
        nodes.append(Node(
            package='rtabmap_sync', executable='rgbd_sync',
            name=f'rgbd_{camera}_sync', output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'approx_sync': True, 'approx_sync_max_interval': 0.02,
                'qos': 2, 'qos_camera_info': 2,
            }],
            remappings=[
                ('rgb/image', f'{prefix}/color/image_raw'),
                ('depth/image', f'{prefix}/depth/image_raw'),
                ('rgb/camera_info', f'{prefix}/camera_info'),
                ('rgbd_image', f'{prefix}/rgbd_image'),
            ],
        ))
    nodes.append(Node(
        package='rtabmap_sync', executable='rgbdx_sync',
        name='rgbd_cameras_sync', output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'rgbd_cameras': 2, 'approx_sync': True,
            'approx_sync_max_interval': 0.02, 'qos': 2,
        }],
        remappings=[
            ('rgbd_image0', '/rgbd_blue/rgbd_image'),
            ('rgbd_image1', '/rgbd_orange/rgbd_image'),
            ('rgbd_images', '/rgbd_images'),
        ],
    ))
    return nodes


def launch_rtabmap(context):
    if not IfCondition(LaunchConfiguration('enable_rtabmap')).evaluate(context):
        return []
    enable_rgbd = IfCondition(LaunchConfiguration('enable_rgbd')).evaluate(context)
    enable_lidar = IfCondition(LaunchConfiguration('enable_lidar')).evaluate(context)
    if not (enable_rgbd or enable_lidar):
        raise RuntimeError('enable_rtabmap requires enable_rgbd:=true or enable_lidar:=true')

    sync_rgbd = IfCondition(LaunchConfiguration('RTABMap_sync')).evaluate(context)
    if sync_rgbd and not enable_rgbd:
        raise RuntimeError('RTABMap_sync requires enable_rgbd:=true')

    # Resolve the optional package only when RTAB-Map is requested.
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('rtabmap_launch'), 'launch', 'rtabmap.launch.py',
        ])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'frame_id': 'frame_assembly',
            'visual_odometry': 'false' if enable_lidar else 'true',
            'icp_odometry': 'true' if enable_lidar else 'false',
            'depth': 'true' if enable_rgbd and not sync_rgbd else 'false',
            'subscribe_rgb': 'true' if enable_rgbd and not sync_rgbd else 'false',
            'subscribe_rgbd': 'true' if sync_rgbd else 'false',
            'rgbd_sync': 'false',
            'subscribe_scan_cloud': 'true' if enable_lidar else 'false',
            'scan_cloud_topic': '/unilidar/cloud',
            'publish_tf_odom': 'true',
            'vo_frame_id': 'odom',
            'odom_topic': '/rtabmap/icp_odom' if enable_lidar else '/rtabmap/visual_odom',
            'rtabmap_args': (
                '--Reg/Strategy 1 --Reg/Force3DoF true '
                '--RGBD/ProximityBySpace true --Grid/Sensor 0 '
                '--Icp/VoxelSize 0.05 --Icp/PointToPlane true'
                if enable_lidar else ''
            ) + (' --Vis/EstimationType 0' if sync_rgbd else ''),
            'rgb_topic': '/rgbd_blue/color/image_raw',
            'depth_topic': '/rgbd_blue/depth/image_raw',
            'camera_info_topic': '/rgbd_blue/camera_info',
            'approx_sync': 'true',
            'qos': '2',
            'rtabmap_viz': 'true',
            'rviz': 'false',
            'database_path': LaunchConfiguration('rtabmap_database_path'),
        }.items(),
    )
    if sync_rgbd:
        # The upstream launch does not expose RGBDImages input arguments.
        # Apply these to mapping, visualization and optional RGB-D odometry.
        return [GroupAction(actions=[
            SetParameter(name='rgbd_cameras', value=0),
            SetRemap(src='rgbd_images', dst='/rgbd_images'),
            rtabmap,
        ])]
    return [rtabmap]


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
        ],
        output='screen',
    )

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        condition=IfCondition(LaunchConfiguration('enable_lidar')),
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
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
            description='Run RTAB-Map with the enabled RGB-D and LiDAR sensors',
        ),
        DeclareLaunchArgument(
            'rtabmap_database_path', default_value='/tmp/rgbd_visual_test.db',
            description='RTAB-Map database to save or resume',
        ),
        DeclareLaunchArgument(
            'enable_rgbd', default_value=LaunchConfiguration('enable_rtabmap'),
            description='Bridge the optional blue and orange RGB-D cameras (also enable it in robot_description)',
        ),
        DeclareLaunchArgument(
            'enable_lidar', default_value=LaunchConfiguration('enable_rtabmap'),
            description='Bridge the optional simulated LiDAR (also enable it in robot_description)',
        ),
        DeclareLaunchArgument(
            'RTABMap_sync', default_value='false',
            description='Synchronize blue and orange RGB-D cameras into /rgbd_images',
        ),
        OpaqueFunction(function=launch_rgbd_sync),
        OpaqueFunction(function=launch_rtabmap),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('startup'), 'launch', 'rgbd_sim.launch.py',
            ])),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=IfCondition(LaunchConfiguration('enable_rgbd')),
        ),
        mock_actuator_feedback,
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[PathJoinSubstitution([FindPackageShare('startup'), 'models'])]
        ),
        bridge,
        lidar_bridge,
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
