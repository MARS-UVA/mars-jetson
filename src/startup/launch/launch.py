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

    # Zenoh router. Every rmw_zenoh_cpp participant on this host connects through
    # it, so it has to be up before anything else -- starting it here replaces the
    # manual "ros2 run rmw_zenoh_cpp rmw_zenohd" that used to live in
    # setup_terminal.sh (kept there, commented, as a fallback for by-hand starts).
    #
    # Deliberately no parameters=[]: rmw_zenohd is not an rclcpp node (it does not
    # link rclcpp at all), so launch_ros's "--ros-args -p ..." argv is silently
    # ignored. It reads its config only from ZENOH_ROUTER_CONFIG_URI, or from
    # ZENOH_CONFIG_OVERRIDE for individual keys. The packaged default already
    # listens on tcp/[::]:7447, which is what we want, so we pass nothing.
    nodes.append(Node(
        package="rmw_zenoh_cpp",
        executable="rmw_zenohd",
        name="rmw_zenohd",
        output="screen",
        respawn=True,
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

