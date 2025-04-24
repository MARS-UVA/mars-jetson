import os
from ament_index_python import get_package_share_directory
import launch
import launch_ros.actions


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('localization_sim'),
        'config',
        'localization_sim.yaml'
    )
    logger = launch.substitutions.LaunchConfiguration('log_level')
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'log_level',
            default_value=['info'],
            description='Logging level'
        ),
        launch_ros.actions.SetParameter('use_sim_time', True),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--frame-id', 'world',
                       '--child-frame-id', 'odom',
                       '--x', '0',
                       '--y', '0',
                       '--z', '0',
                       '--qx', '0',
                       '--qy', '0',
                       '--qz', '0',
                       '--qw', '1']),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--frame-id', 'base_link',
                       '--child-frame-id', 'camera_optical',
                       '--x', '0.006335',
                       '--y', '0.04254',
                       '--z', '0.008895',
                       '--qx', '-0.5',
                       '--qy', '0.5',
                       '--qz', '-0.5',
                       '--qw', '0.5']),
        launch_ros.actions.Node(
            package='state_filter',
            executable='robot_ukf_node',
            parameters=[config],
            arguments=['--ros-args', '--log-level', logger]
        ),
        launch_ros.actions.Node(
            package='apriltag_nodes',
            executable='estimation_node',
            parameters=[config],
            arguments=['--ros-args', '--log-level', logger]
        ),
        # launch_ros.actions.Node(
        #     package='localization_sim',
        #     executable='pose_listener',
        #     parameters=[config],
        #     arguments=['--ros-args', '--log-level', logger]
        # ),
        launch_ros.actions.Node(
            package='localization_sim',
            executable='simulated_data',
            parameters=[config],
            arguments=['--ros-args', '--log-level', logger]
        ),
  ])
