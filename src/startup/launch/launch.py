from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, EnvironmentVariable, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition

def generate_launch_description():
    backend = LaunchConfiguration("robot_backend")
    enable_rtabmap = LaunchConfiguration("enable_rtabmap")
    enable_rgbd = LaunchConfiguration("enable_rgbd")
    enable_lidar = LaunchConfiguration("enable_lidar")
    control_station_ip = LaunchConfiguration("control_station_ip")

    backend_arg = DeclareLaunchArgument(
        "robot_backend",
        default_value="serial",
        description="Backend for the robot hardware interface (serial, gazebo, mock)",
    )
    control_station_ip_arg = DeclareLaunchArgument(
        "control_station_ip",
        default_value=EnvironmentVariable("CONTROL_STATION_IP", default_value="192.168.50.60"),
        description="IP address of the control station for network communication",
    )
    args = [backend_arg, control_station_ip_arg, DeclareLaunchArgument(
        'enable_rtabmap', default_value='false',
        description='Run RTAB-Map with the enabled RGB-D and LiDAR sensors in Gazebo',
    ), DeclareLaunchArgument(
        'rtabmap_database_path', default_value='/tmp/rgbd_visual_test.db',
        description='RTAB-Map database to save or resume',
    ), DeclareLaunchArgument(
        'enable_rgbd', default_value=enable_rtabmap,
        description='Add blue and orange RGB-D cameras when using the Gazebo backend',
    ), DeclareLaunchArgument(
        'enable_lidar', default_value=enable_rtabmap,
        description='Enable the simulated LiDAR when using the Gazebo backend',
    )]

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
        " enable_rgbd:=", enable_rgbd,
        " enable_lidar:=", enable_lidar,
    ])


    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'use_sim_time': ParameterValue(
                PythonExpression(["'", backend, "' == 'gazebo'"]), value_type=bool),
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
            'robot_backend': backend,
            'enable_rgbd': enable_rgbd,
            'enable_lidar': enable_lidar,
            'enable_rtabmap': enable_rtabmap,
            'rtabmap_database_path': LaunchConfiguration('rtabmap_database_path'),
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", backend, "' == 'gazebo'"])
        )
    ))

    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([startup_pkg, 'launch', 'lidar.launch.py'])),
        condition=IfCondition(
            PythonExpression(["'", backend, "' == 'serial'"])
        )
    ))

    return LaunchDescription([
        *args,
        SetEnvironmentVariable(
            name="CONTROL_STATION_IP",
            value=control_station_ip,
        ),
        *nodes,
    ])
