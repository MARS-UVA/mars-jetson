import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    dig = Node(
        package='digdump_actions',
        executable='dig_node',
        name='dig_node',
        output='screen',
        parameters=[{
            'actuator_speed_aerial': 1.0,
            'actuator_speed_ground': 0.75,
            'dig_speed': 0.75,
            'dig_time': 3.0,
            'actuator_extend_length_aerial': 0.69,
            'actuator_extend_length_ground': 0.9,
            'actuator_raise_target': 0.6,
        }],
        respawn=True
    )
    dump = Node(
        package='digdump_actions',
        executable='dump_node',
        name='dump_node',
        output='screen',
        parameters=[{
            'dump_speed': 1.0,
            'drive_speed': 0.25,
            'dump_time': 6.0,
            'move_time': 6.0,
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
    control_config = os.path.join(
        get_package_share_directory('robot_super_intelligence'), 'config', 'control.yaml')
    command_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='command_mux',
        remappings=[('cmd_vel_out', 'cmd_vel')],
        parameters=[control_config],
        output='screen',
        respawn=True,
    )
    with open(control_config, encoding='utf-8') as config_file:
        arm_config = yaml.safe_load(config_file)['/**/arm_drum_mux']['ros__parameters']
    arm_drum_mux = Node(
        package='topic_tools',
        executable='mux',
        name='arm_drum_mux',
        arguments=[arm_config['output_topic'], *arm_config['input_topics']],
        parameters=[control_config],
        output='screen',
        respawn=True,
    )
    state_manager = Node(
        package='robot_super_intelligence',
        executable='state_manager',
        name='robot_super_intelligence',
        parameters=[control_config],
        output='screen',
        respawn=True,
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
    
    apriltag = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        remappings=[
            ('image_rect', '/front_camera/image_raw'),
            ('camera_info', '/front_camera/camera_info'),
        ],
        parameters=[
            os.path.join(get_package_share_directory('startup'), 'config', 'apriltag.yaml'),
            {
                'use_sim_time': ParameterValue(PythonExpression([
                    "'", LaunchConfiguration('robot_backend', default='serial'),
                    "' == 'gazebo'",
                ]), value_type=bool),
            },
        ],
        output='screen'
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
        dig,
        dump,
        controller,
        command_mux,
        arm_drum_mux,
        state_manager,
        gstreamer,
        twist_stamper,
        apriltag,
        *controllers
    ])

