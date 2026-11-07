import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import (
    LaunchConfiguration, NotSubstitution, PathJoinSubstitution,
)
from launch_ros.actions import SetParameter, SetRemap
from launch_ros.descriptions import ComposableNode

def launch_rgbd_sync(context):
    if not IfCondition(LaunchConfiguration('rtabmap_sync')).evaluate(context):
        return []
    if not IfCondition(LaunchConfiguration('enable_rgbd')).evaluate(context):
        raise RuntimeError('rtabmap_sync requires enable_rgbd:=true')

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
                ('rgb/image', f'{prefix}/rgb/image_raw'),
                ('depth/image', f'{prefix}/stereo/image_raw'),
                ('rgb/camera_info', f'{prefix}/rgb/camera_info'),
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

    sync_rgbd = IfCondition(LaunchConfiguration('rtabmap_sync')).evaluate(context)
    if sync_rgbd and not enable_rgbd:
        raise RuntimeError('rtabmap_sync requires enable_rgbd:=true')

    # Resolve the optional package only when RTAB-Map is requested.
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('rtabmap_launch'), 'launch', 'rtabmap.launch.py',
        ])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'frame_id': 'base_link',
            'visual_odometry': 'false' if enable_lidar else 'true',
            'icp_odometry': 'true' if enable_lidar else 'false',
            'depth': 'true' if enable_rgbd and not sync_rgbd else 'false',
            'subscribe_rgb': 'true' if enable_rgbd and not sync_rgbd else 'false',
            'subscribe_rgbd': 'true' if sync_rgbd else 'false',
            'rgbd_sync': 'false',
            'rgbd_cameras': '2' if sync_rgbd else '1',
            'rgbd_image_topic': '/rgbd_blue/rgbd_image',
            'rgbd_images_topic': '/rgbd_images',
            'subscribe_scan_cloud': 'true' if enable_lidar else 'false',
            'scan_cloud_topic': '/unilidar/cloud_filtered',
            'publish_tf_odom': 'true',
            'odom_frame_id': 'odom',
            'vo_frame_id': 'odom',
            'odom_topic': '/rtabmap/icp_odom' if enable_lidar else '/rtabmap/visual_odom',
            'rtabmap_args': (
                '--Reg/Strategy 1 '
                '--Reg/Force3DoF true '
                '--Optimizer/Slam2D true '
                '--Rtabmap/PublishTf true '
                '--Rtabmap/TfDelay 0.05 '
                '--Rtabmap/DetectionRate 2.0 '
                # Suppress false loop closures on uniform concrete floors
                '--Rtabmap/LoopThr 0.15 '
                '--Vis/MinInliers 15 '
                '--RGBD/ProximityBySpace true '
                '--RGBD/ProximityByTime false '
                # 3D Grid & Ground retention
                '--Grid/Sensor 1 '
                '--Grid/3D true '
                '--Grid/FromDepth true '
                '--Grid/GroundIsObstacle true '           # Retains floor points in /rtabmap/cloud_map
                '--Grid/NormalsSegmentation false '
                '--Grid/RangeMin 0.35 '
                '--Grid/RangeMax 3.0 '                    # Truncates noisy distant stereo returns to stop the floor curling
                '--Grid/CellSize 0.05 '
                # Robot physical footprint envelope in base_link
                # '--Grid/FootprintLength 1.80 --Grid/FootprintWidth 0.76 '
                # '--Grid/FootprintHeight 0.50 '
                '--Icp/VoxelSize 0.05 --Icp/PointToPlane true'
                if enable_lidar else ''
            ) + (' --Vis/EstimationType 0' if sync_rgbd else ''),
            'rgb_topic': '/rgbd_blue/rgb/image_raw',
            'depth_topic': '/rgbd_blue/stereo/image_raw',
            'camera_info_topic': '/rgbd_blue/rgb/camera_info',
            'approx_sync': 'true',
            'qos': '2',
            'rtabmap_viz': NotSubstitution(LaunchConfiguration('no_gui', default='false')),
            'rviz': 'false',
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
    ros2_control_params_file = os.path.join(
        get_package_share_directory("startup"),
        "config",
        "controllers.yaml"
    )

    teleop = Node(
        package='teleop',
        executable='teleop',
        name='teleop',
        output='screen',
        parameters=[{
            'linear_axis': 'left_y',
            'turn_axis': 'left_x_inverted',
            'full_forward_magnitude': 0.6,
            'turn_speed_scale': 2.0,
            'deadband': 0.05
        }],
        arguments=['--ros-args', '--log-level', 'WARN'],
        respawn=True
    )
    dig = Node(
        package='digdump_actions',
        executable='dig_node',
        name='dig_node',
        output='screen',
        parameters=[{
            'actuator_speed_aerial': 1.0,
            'actuator_speed_ground': 0.75,
            'dig_speed': 0.75,
            'dig_time': 3.0,
            'actuator_extend_length_aerial': 0.69,
            'actuator_extend_length_ground': 0.9,
            'actuator_raise_target': 0.6,
        }],
        respawn=True
    )
    dump = Node(
        package='digdump_actions',
        executable='dump_node',
        name='dump_node',
        output='screen',
        parameters=[{
            'dump_speed': 1.0,
            'drive_speed': 0.25,
            'dump_time': 6.0,
            'move_time': 6.0,
        }],
        respawn=True
    )
    controller = Node(
        package='robot_controller',
        executable='robot_controller',
        name='robot_controller',
        output='screen',
        arguments=['--ros-args', '--log-level', 'WARN'],
        respawn=True
    )
    cmd_vel_mux = Node(
        package='topic_tools',
        executable='mux',
        name='cmd_vel_mux',
        output='screen',
        arguments=['/cmd_vel', '/cmd_vel/teleop', '/cmd_vel/dig', '/cmd_vel/dump', '/cmd_vel/navigation'],
        respawn=True
    )
    arm_drum_mux = Node(
        package='topic_tools',
        executable='mux',
        name='arm_drum_mux',
        output='screen',
        arguments=['/arm_drum_controller/commands', '/arm_drum_control/teleop', '/arm_drum_control/dig', '/arm_drum_control/dump', '/arm_drum_control/stop'],
        respawn=True
    )
    robot_state_controller = Node(
        package='robot_state_controller',
        executable='robot_state_controller',
        name='robot_state_controller',
        output='screen',
        arguments=['--ros-args', '--log-level', 'WARN'],
        respawn=True
    )
    # Old WebRTC streamers. Cameras now reach the dashboard as H.264 topics over
    # rosbridge; this stays available with legacy_webrtc:=true until it is removed.
    gstreamer = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('gstreamer'), 'gstreamer_launch.py')]),
      condition=IfCondition(LaunchConfiguration('legacy_webrtc', default='false')),
    )

    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='twist_stamper',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(PythonExpression([
                "'", LaunchConfiguration('robot_backend', default='serial'),
                "' == 'gazebo'",
            ]), value_type=bool),
        }],
        respawn=True,
        remappings=[
            ('/cmd_vel_in', '/cmd_vel'),
            ('/cmd_vel_out', '/base_controller/cmd_vel')
        ]
    )
    
    front_cropbox_config = os.path.join(
        get_package_share_directory('startup'),
        'config',
        'front_arm_cropbox.yaml'
    )
    
    rear_arm_cropbox_config = os.path.join(
        get_package_share_directory('startup'),
        'config',
        'rear_arm_cropbox.yaml'
    )
    
    lidar_cropbox = ComposableNodeContainer(
        name='pcl_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='pcl_ros',
                plugin='pcl_ros::CropBox',
                name='crop_box_filter',
                remappings=[
                    ('/input', '/unilidar/cloud_no_front_arm'),
                    ('/output', '/unilidar/cloud_filtered')
                ],
                parameters=[rear_arm_cropbox_config]
            ),
            ComposableNode(
                package='pcl_ros',
                plugin='pcl_ros::CropBox',
                name='crop_box_filter',
                remappings=[
                    ('/input', '/unilidar/cloud'),
                    ('/output', '/unilidar/cloud_no_front_arm')
                ],
                parameters=[front_cropbox_config]
            )
        ]
    )
    
    front_rgbd_cropbox_config = os.path.join(
        get_package_share_directory('startup'), 'config', 'front_arm_cropbox_rgbd.yaml',
    )
    rear_rgbd_cropbox_config = os.path.join(
        get_package_share_directory('startup'), 'config', 'rear_arm_cropbox_rgbd.yaml',
    )
    
    rgbd_cropbox = ComposableNodeContainer(
        name='pcl_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='pcl_ros',
                plugin='pcl_ros::CropBox',
                name='rgbd_rear_crop_box_filter',
                remappings=[
                    ('/input', '/rgbd_orange/points'),
                    ('/output', '/rgbd_orange/points_filtered'),
                ],
                parameters=[rear_rgbd_cropbox_config]
            ),
            ComposableNode(
                package='pcl_ros',
                plugin='pcl_ros::CropBox',
                name='rgbd_front_crop_box_filter',
                remappings=[
                    ('/input', '/rgbd_blue/points'),
                    ('/output', '/rgbd_blue/points_filtered'),
                ],
                parameters=[front_rgbd_cropbox_config]
            ),
        ]
    )

    controllers = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                ["/", "controller_manager"],
                '--param-file',
                ros2_control_params_file,
            ],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "base_controller",
                "--controller-manager",
                ["/", "controller_manager"],
                '--param-file',
                ros2_control_params_file,
            ],
            namespace="/",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "arm_drum_controller",
                "--controller-manager",
                ["/", "controller_manager"],
                '--param-file',
                ros2_control_params_file,
            ],
            namespace="/"
        )
    ]


    return LaunchDescription([
        teleop, 
        dig,
        dump,
        controller,
        cmd_vel_mux,
        arm_drum_mux,
        robot_state_controller,
        gstreamer,
        twist_stamper,
        lidar_cropbox,
        rgbd_cropbox,
        OpaqueFunction(function=launch_rgbd_sync),
        OpaqueFunction(function=launch_rtabmap),
        *controllers
    ])

