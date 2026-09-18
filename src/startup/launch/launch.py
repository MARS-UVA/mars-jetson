from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, EnvironmentVariable, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    backend = LaunchConfiguration("robot_backend")
    control_station_ip = LaunchConfiguration("control_station_ip")

    backend_arg = DeclareLaunchArgument(
        "robot_backend",
        default_value="serial",
        description="Backend for the robot hardware interface (serial, gazebo, mock)",
    )
    control_station_ip_arg = DeclareLaunchArgument(
        "control_station_ip",
        default_value="192.168.50.60",
        description="IP address of the control station for network communication",
    )
    args = [backend_arg, control_station_ip_arg]

    startup_pkg = FindPackageShare("startup")

    nodes = []

    nodes.append(Node(
        package="rmw_zenoh_cpp",
        executable="rmw_zenohd",
        name="rmw_zenohd",
        output="screen",
        parameters=[
            {"zenoh_router_port": 7447},
            {"zenoh_router_log_level": "info"}
        ]
    ))

    # Process the xacro file and create the robot description
    robot_description = Command([
        "xacro ",
        PathJoinSubstitution([
            startup_pkg,
            "urdf",
            "robot",
            "urdf",
            "robot.urdf.xacro"
        ]),
        " robot_backend:=", backend,
        " camera_update_rate:=30",
    ])


    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }],
    ))

    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([startup_pkg, 'launch', 'robot.launch.py'])),
        launch_arguments={
            'robot_backend': backend
        }.items()
    ))

    nodes.append(Node(
        package='serial_node',
        executable='op_reader',
        name='serial_node',
        output='screen',
        arguments=['--ros-args', '--log-level', 'WARN'],
        parameters=[
            {'mock_serial': EnvironmentVariable('MOCK_SERIAL', default_value='0')}
        ],
        respawn=True,
        condition=IfCondition(
            PythonExpression(["'", backend, "' == 'serial'"])
        )
    ))

    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([startup_pkg, 'launch', 'gazebo.launch.py'])),
        launch_arguments={
            'robot_backend': backend
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", backend, "' == 'gazebo'"])
        )
    ))

    return LaunchDescription([
        SetEnvironmentVariable(
            name="CONTROL_STATION_IP",
            value=control_station_ip,
        ),
        *args,
        *nodes,
    ])

