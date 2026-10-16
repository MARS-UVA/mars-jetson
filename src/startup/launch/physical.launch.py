from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Load the robot description as a parameter, passed in from the parent launch file.
    robot_description = LaunchConfiguration('robot_description')

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

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('startup'), 'launch', 'lidar.launch.py']))
    )

    ld = LaunchDescription([
        controller_manager,
        lidar,
    ])
    return ld
