from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, PythonExpression, NotSubstitution
from launch_ros.actions import Node, SetParameter, SetRemap
from launch_ros.parameter_descriptions import ParameterFile
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.actions import OpaqueFunction, GroupAction

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
                '--Rtabmap/LoopThr 0.15 '
                '--Vis/MinInliers 15 '
                '--RGBD/ProximityBySpace true '
                '--RGBD/ProximityByTime false '
                '--Grid/Sensor 1 '
                '--Grid/3D true '
                '--Grid/FromDepth true '
                '--Grid/GroundIsObstacle false '
                '--Grid/NormalsSegmentation true '
                '--Grid/RangeMin 0.35 '
                '--Grid/RangeMax 3.0 '
                '--Grid/CellSize 0.05 '
                # Robot physical footprint envelope in base_link
                # '--Grid/FootprintLength 1.80 --Grid/FootprintWidth 0.76 '
                # '--Grid/FootprintHeight 0.50 '
                '--Icp/VoxelSize 0.05 --Icp/PointToPlane true'
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

def launch_nav2(context):
    if not IfCondition(LaunchConfiguration('enable_nav2')).evaluate(context):
        return []
    if not IfCondition(LaunchConfiguration('enable_rtabmap')).evaluate(context):
        raise RuntimeError('enable_nav2 requires enable_rtabmap:=true')
    # Resolve Nav2 packages only inside the enabled navigation launch.
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('robot_navigation'), 'launch', 'navigation.launch.py',
        ])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time', default='false'),
            'odom_topic': IfCondition(LaunchConfiguration('enable_lidar')).perform(context) and '/rtabmap/icp_odom' or '/rtabmap/visual_odom',
            'enable_local_lidar': LaunchConfiguration('enable_local_lidar', default='false'),
        }.items(),
    )]


def generate_launch_description():
    # Load the robot description as a parameter, passed in from the parent launch file.
    robot_description = LaunchConfiguration('robot_description')
    color_cameras = LaunchConfiguration('color_cameras')
    enable_rgbd = LaunchConfiguration('enable_rgbd')

    controller_config = ParameterFile(
        PathJoinSubstitution([
            FindPackageShare("startup"),
            "config",
            "controllers.yaml"
        ]),
        allow_substs=True
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": ParameterValue(robot_description, value_type=str)},
            controller_config
        ],
        output="screen",
    )

    # Real cameras (v4l2_camera, H.264 via ffmpeg_image_transport). Hardware only:
    # Gazebo publishes the same topics from its image_bridge.
    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('startup'), 'launch', 'cameras.launch.py'])),
        launch_arguments={
            'robot_description': robot_description,
            'color_cameras': color_cameras,
        }.items()
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('startup'), 'launch', 'lidar.launch.py']))
    )

    oakd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('startup'), 'launch', 'oakd.startup.py'])),
        condition=IfCondition(
            PythonExpression(["'", enable_rgbd, "' == 'true'"])
        )
    )

    ld = LaunchDescription([
        controller_manager,
        cameras,
        lidar,
        oakd,
        OpaqueFunction(function=launch_rgbd_sync),
        OpaqueFunction(function=launch_rtabmap),
        OpaqueFunction(function=launch_nav2),
    ])
    return ld
