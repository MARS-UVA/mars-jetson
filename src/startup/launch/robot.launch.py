import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    ros2_control_params_file = os.path.join(
        get_package_share_directory("startup"),
        "config",
        "controllers.yaml"
    )

    teleop = Node(
        package='teleop',
        executable='teleop',
        name='teleop',
        output='screen',
        parameters=[{
            'linear_axis': 'left_y',
            'turn_axis': 'left_x_inverted',
            'full_forward_magnitude': 0.6,
            'turn_speed_scale': 2.0,
            'deadband': 0.05
        }],
        arguments=['--ros-args', '--log-level', 'WARN'],
        respawn=True
    )
    digdump = Node(
        package='digdump',
        executable='action_server',
        name='digdump',
        output='screen',
        parameters=[{
            'actuator_speed_aerial': 1.0,
            'actuator_speed_ground': 0.75,
            'dig_speed': 0.75,
            'dump_speed': 1.0,
            'drive_speed': 0.25,
            'dig_time': 3.0,
            'dump_time': 6.0,
            'move_time': 6.0,
            'actuator_extend_length_aerial': 0.69,
            'actuator_extend_length_ground': 0.9,
        }],
        respawn=True
    )
    controller = Node(
        package='robot_controller',
        executable='robot_controller',
        name='robot_controller',
        output='screen',
        arguments=['--ros-args', '--log-level', 'WARN'],
        respawn=True
    )
    cmd_vel_mux = Node(
        package='topic_tools',
        executable='mux',
        name='cmd_vel_mux',
        output='screen',
        arguments=['/cmd_vel', '/cmd_vel/teleop', '/cmd_vel/autonomy', '/cmd_vel/navigation'],
        respawn=True
    )
    arm_drum_mux = Node(
        package='topic_tools',
        executable='mux',
        name='arm_drum_mux',
        output='screen',
        arguments=['/arm_drum_controller/commands', '/arm_drum_control/teleop', '/arm_drum_control/autonomy', '/arm_drum_control/stop'],
        respawn=True
    )
    robot_state_controller = Node(
        package='robot_state_controller',
        executable='robot_state_controller',
        name='robot_state_controller',
        output='screen',
        arguments=['--ros-args', '--log-level', 'WARN'],
        respawn=True
    )
    # Old WebRTC streamers. Cameras now reach the dashboard as H.264 topics over
    # rosbridge; this stays available with legacy_webrtc:=true until it is removed.
    gstreamer = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('gstreamer'), 'gstreamer_launch.py')]),
      condition=IfCondition(LaunchConfiguration('legacy_webrtc', default='false')),
    )

    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='twist_stamper',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(PythonExpression([
                "'", LaunchConfiguration('robot_backend', default='serial'),
                "' == 'gazebo'",
            ]), value_type=bool),
        }],
        respawn=True,
        remappings=[
            ('/cmd_vel_in', '/cmd_vel'),
            ('/cmd_vel_out', '/base_controller/cmd_vel')
        ]
    )
    
    front_cropbox_config = os.path.join(
        get_package_share_directory('startup'),
        'config',
        'front_arm_cropbox.yaml'
    )

    front_arm_cropbox = ComposableNodeContainer(
        name='pcl_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='pcl_ros',
                plugin='pcl_ros::CropBox',
                name='crop_box_filter',
                remappings=[
                    ('/input', '/unilidar/cloud'),
                    ('/output', '/unilidar/cloud_no_front_arm')
                ],
                parameters=[front_cropbox_config]
            )
        ]
    )

    controllers = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                ["/", "controller_manager"],
                '--param-file',
                ros2_control_params_file,
            ],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "base_controller",
                "--controller-manager",
                ["/", "controller_manager"],
                '--param-file',
                ros2_control_params_file,
            ],
            namespace="/",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "arm_drum_controller",
                "--controller-manager",
                ["/", "controller_manager"],
                '--param-file',
                ros2_control_params_file,
            ],
            namespace="/"
        )
    ]


    return LaunchDescription([
        teleop, 
        digdump,
        controller,
        cmd_vel_mux,
        arm_drum_mux,
        robot_state_controller,
        gstreamer,
        twist_stamper,
        front_arm_cropbox,
        *controllers
    ])

