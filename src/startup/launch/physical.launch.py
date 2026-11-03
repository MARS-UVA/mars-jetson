from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition

from launch_ros.substitutions import FindPackageShare

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
        oakd
    ])
    return ld
