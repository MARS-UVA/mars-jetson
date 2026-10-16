from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, EnvironmentVariable, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition


def launch_nav2(context):
    if not IfCondition(LaunchConfiguration('enable_nav2')).evaluate(context):
        return []
    if LaunchConfiguration('robot_backend').perform(context) != 'gazebo':
        raise RuntimeError('enable_nav2 currently requires robot_backend:=gazebo')
    for argument in ('enable_rtabmap', 'enable_rgbd', 'enable_lidar'):
        if not IfCondition(LaunchConfiguration(argument)).evaluate(context):
            raise RuntimeError(f'enable_nav2 requires {argument}:=true')
    # Resolve Nav2 packages only inside the enabled navigation launch.
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('startup'), 'launch', 'nav2.launch.py',
        ])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time', default='true'),
            'odom_topic': '/rtabmap/icp_odom',
            'enable_local_lidar': LaunchConfiguration('enable_local_lidar', default='false'),
        }.items(),
    )]


def generate_launch_description():
    backend = LaunchConfiguration("robot_backend")
    enable_rtabmap = LaunchConfiguration("enable_rtabmap")
    enable_rgbd = LaunchConfiguration("enable_rgbd")
    enable_lidar = LaunchConfiguration("enable_lidar")
    control_station_ip = LaunchConfiguration("control_station_ip")
    serial_port = LaunchConfiguration("serial_port")

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
        'enable_local_lidar', default_value='false',
        description='Use filtered LiDAR alongside RGB-D in the local navigation costmap',
    ), DeclareLaunchArgument(
        'enable_nav2', default_value='false',
        description='Run optional Nav2 navigation with RTAB-Map and all obstacle sensors in Gazebo',
    ), DeclareLaunchArgument(
        'enable_rtabmap', default_value='false',
        description='Run RTAB-Map with the enabled RGB-D and LiDAR sensors in Gazebo',
    ), DeclareLaunchArgument(
        'rtabmap_sync', default_value='false',
        description='Synchronize blue and orange RGB-D cameras for RTAB-Map',
    ), DeclareLaunchArgument(
        'rtabmap_database_path', default_value='/tmp/rgbd_visual_test.db',
        description='RTAB-Map database to save or resume',
    ), DeclareLaunchArgument(
        'enable_rgbd', default_value=enable_rtabmap,
        description='Add blue and orange RGB-D cameras when using the Gazebo backend',
    ), DeclareLaunchArgument(
        'enable_lidar', default_value=enable_rtabmap,
        description='Enable the simulated LiDAR when using the Gazebo backend',
    ), DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0',
        description='Serial port for the robot hardware interface when using the serial backend',
    )]

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
        " enable_rgbd:=", enable_rgbd,
        " enable_lidar:=", enable_lidar,
        " serial_port:=", serial_port
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

    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([startup_pkg, 'launch', 'gazebo.launch.py'])),
        launch_arguments={
            'robot_backend': backend,
            'enable_rgbd': enable_rgbd,
            'enable_lidar': enable_lidar,
            'enable_rtabmap': enable_rtabmap,
            'rtabmap_sync': LaunchConfiguration('rtabmap_sync'),
            'rtabmap_database_path': LaunchConfiguration('rtabmap_database_path'),
            'rviz_config': PathJoinSubstitution([
                startup_pkg, 'config', PythonExpression([
                    "'nav2.rviz' if '", LaunchConfiguration('enable_nav2'),
                    "'.lower() in ('true', '1') else 'RGBDRViz.rviz'",
                ]),
            ]),
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", backend, "' == 'gazebo'"])
        )
    ))

    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([startup_pkg, 'launch', 'physical.launch.py'])),
        launch_arguments={
            'robot_description': robot_description,
        }.items(),
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
        OpaqueFunction(function=launch_nav2),
    ])
