"""Navigation servers using an externally supplied map, TF, and odometry."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def launch_navigation(context):
    # Keep Nav2 imports optional for the ordinary robot startup path.
    from nav2_common.launch import RewrittenYaml

    configured_params = ParameterFile(RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        param_rewrites={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'odom_topic': LaunchConfiguration('odom_topic'),
        },
        convert_types=True,
    ))
    servers = [
        ('nav2_controller', 'controller_server'),
        ('nav2_smoother', 'smoother_server'),
        ('nav2_planner', 'planner_server'),
        ('nav2_behaviors', 'behavior_server'),
        ('nav2_bt_navigator', 'bt_navigator'),
        ('nav2_velocity_smoother', 'velocity_smoother'),
    ]
    nodes = []
    for package, executable in servers:
        remappings = []
        if executable in ('controller_server', 'behavior_server'):
            remappings = [('cmd_vel', '/cmd_vel/navigation_raw')]
        elif executable == 'velocity_smoother':
            remappings = [
                ('cmd_vel', '/cmd_vel/navigation_raw'),
                ('cmd_vel_smoothed', '/cmd_vel/navigation'),
            ]
        nodes.append(Node(
            package=package, executable=executable, name=executable,
            output='screen', parameters=[configured_params], remappings=remappings,
        ))
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'node_names': [executable for _, executable in servers],
        }],
    ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('odom_topic', default_value='/rtabmap/icp_odom'),
        DeclareLaunchArgument(
            'params_file', default_value=PathJoinSubstitution([
                FindPackageShare('startup'), 'config', 'nav2.yaml',
            ]),
        ),
        OpaqueFunction(function=launch_navigation),
    ])
