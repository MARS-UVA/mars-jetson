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
    planner_choices = {
        'smac': 'SmacPlanner',
        'theta_star': 'ThetaStar',
        'smoothed_lazy_theta_star': 'LazyThetaStar',
    }
    planner_choice = LaunchConfiguration(
        'global_planner', default='smoothed_lazy_theta_star').perform(context)
    if planner_choice not in planner_choices:
        raise RuntimeError(
            'global_planner must be one of: smac, theta_star, smoothed_lazy_theta_star')
    planner_id = planner_choices[planner_choice]

    def parameters(filename):
        rewrites = {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'odom_topic': LaunchConfiguration('odom_topic'),
        }
        if filename == 'planner.yaml':
            rewrites['planner_server.ros__parameters.planner_plugins'] = f'[{planner_id}]'
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
            package='robot_navigation', executable='navigation_cloud_filter',
            name='navigation_cloud_filter', output='screen',
            parameters=[configured_params['local_costmap.yaml']],
        ))
    for package, executable, filenames in servers:
        remappings = []
        server_parameters = [configured_params[filename] for filename in filenames]
        if executable == 'bt_navigator':
            server_parameters.append({
                'default_nav_to_pose_bt_xml': PathJoinSubstitution([
                    FindPackageShare('robot_navigation'), 'behavior_trees',
                    'navigate_to_pose_thetastar.xml',
                ]),
            })
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
            parameters=server_parameters,
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
        DeclareLaunchArgument(
            'global_planner', default_value='smoothed_lazy_theta_star',
            description=(
                'Global planner: smac, theta_star, or smoothed_lazy_theta_star'),
        ),
        DeclareLaunchArgument('odom_topic', default_value='/rtabmap/icp_odom'),
        DeclareLaunchArgument(
            'config_dir', default_value=PathJoinSubstitution([
                FindPackageShare('robot_navigation'), 'config',
            ]),
            description='Directory containing the seven Nav2 parameter files',
        ),
        OpaqueFunction(function=launch_navigation),
    ])
