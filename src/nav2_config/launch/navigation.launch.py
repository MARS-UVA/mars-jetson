"""Navigation servers using an externally supplied map, TF, and odometry."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def launch_navigation(context):
    local_lidar = IfCondition(
        LaunchConfiguration('enable_local_lidar', default='false')).evaluate(context)

    def parameters(filename):
        rewrites = {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'odom_topic': LaunchConfiguration('odom_topic'),
        }
        if filename == 'local_costmap.yaml':
            rewrites[
                'local_costmap.local_costmap.ros__parameters.voxel_layer.observation_sources'
            ] = 'rgbd_blue rgbd_orange lidar' if local_lidar else 'rgbd_blue rgbd_orange'
        return ParameterFile(RewrittenYaml(
            source_file=PathJoinSubstitution([
                LaunchConfiguration('config_dir'), filename,
            ]),
            param_rewrites=rewrites,
            convert_types=True,
        ))

    configured_params = {
        filename: parameters(filename) for filename in (
            'planner.yaml', 'controller.yaml', 'global_costmap.yaml',
            'local_costmap.yaml', 'bt_navigator.yaml', 'behaviors.yaml',
            'smoothers.yaml',
        )
    }
    # Costmaps are created by their owning servers and need their own namespaces.
    servers = [
        ('nav2_controller', 'controller_server', ['controller.yaml', 'local_costmap.yaml']),
        ('nav2_smoother', 'smoother_server', ['smoothers.yaml']),
        ('nav2_planner', 'planner_server', ['planner.yaml', 'global_costmap.yaml']),
        ('nav2_behaviors', 'behavior_server', ['behaviors.yaml']),
        ('nav2_bt_navigator', 'bt_navigator', ['bt_navigator.yaml']),
        ('nav2_velocity_smoother', 'velocity_smoother', ['smoothers.yaml']),
    ]
    nodes = []
    if local_lidar:
        nodes.append(Node(
            package='nav2_config', executable='navigation_cloud_filter',
            name='navigation_cloud_filter', output='screen',
            parameters=[configured_params['local_costmap.yaml']],
        ))
    for package, executable, filenames in servers:
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
            output='screen',
            parameters=[configured_params[filename] for filename in filenames],
            remappings=remappings,
        ))
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'node_names': [executable for _, executable, _ in servers],
        }],
    ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('enable_local_lidar', default_value='false'),
        DeclareLaunchArgument('odom_topic', default_value='/rtabmap/icp_odom'),
        DeclareLaunchArgument(
            'config_dir', default_value=PathJoinSubstitution([
                FindPackageShare('nav2_config'), 'config',
            ]),
            description='Directory containing the seven Nav2 parameter files',
        ),
        OpaqueFunction(function=launch_navigation),
    ])
