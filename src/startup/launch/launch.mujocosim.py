import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import PathJoinSubstitution
from launch.actions import Shutdown
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    mujoco_model_path = "/tmp/mujoco"
    mujoco_model_file = os.path.join(mujoco_model_path, "main.xml")
    rviz = LaunchConfiguration("rviz", default="true")

    robot_xacro_filepath = os.path.join(
        get_package_share_directory("startup"),
        "urdf",
        "robot",
        "urdf",
        "robot.urdf.xacro",
    )

    # Process the xacro file and create the robot description
    robot_description = xacro.process_file(
            robot_xacro_filepath,
            mappings={
                "name": "mars_robot",
                "use_sim": "true",
            }
        ).toprettyxml(indent="  ")

    additional_files = []
    additional_files.append(os.path.join(get_package_share_directory("mujoco_ros2_control"), "mjcf", "scene.xml"))

    xacro2mjcf = Node(
        package="mujoco_ros2_control",
        executable="robot_description_to_mjcf.sh",
        output="both",
        emulate_tty=True,
        arguments=[
            "--robot_description",
            robot_description,
            "--scene",
            PathJoinSubstitution([get_package_share_directory("startup"), "urdf", "scene_info.xml"]),
            "--publish_topic",
            "/mujoco_robot_description",
        ],   
    )

    # Define the robot state publisher node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            { "robot_description": robot_description}
        ],
    )

    ros2_control_params_file = os.path.join(
        get_package_share_directory("startup"),
        "config",
        "controllers.yaml"
    )

    mujoco_plugins_file = os.path.join(
        get_package_share_directory("startup"),
        "config",
        "mujoco_plugins.yaml"
    )

    mujoco = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        parameters=[
            { "use_sim_time": True},
            { "robot_description": robot_description},
            {"simulation_frequency": 500.0},
            {"realtime_factor": 1.0},
            {"robot_model_path": mujoco_model_file},
            {"show_gui": True},
            ros2_control_params_file,
            mujoco_plugins_file,
        ],
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
        ],
        on_exit=Shutdown()
    )

    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            ["/", "controller_manager"],
            '--param-file',
            ros2_control_params_file,
        ],
    )

    base_controller_spawner = Node(
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
    )

    arm_drum_controller_spawner = Node(
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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('startup'), 'config', 'robot.rviz')],
        condition=IfCondition(rviz)
    )

    load_controllers = RegisterEventHandler(
        OnProcessStart(
            target_action=mujoco,
            on_start=[
                LogInfo(msg="Starting joint state broadcaster..."),
                load_joint_state_broadcaster,
                base_controller_spawner,
                arm_drum_controller_spawner,
            ],
        )
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
                    'actuator_speed': 1.0,
                    'dig_speed': 0.75,
                    'dump_speed': 1.0,
                    'drive_speed': 0.25,
                    'dig_arm_movement_time': 20.0,
                    'dig_time': 3.0,
                    'dump_time': 6.0,
                    'move_time': 5.0,
                }],
                respawn=True
            )
    network_client = Node(
                package='network_communication',
                executable='udp_client',
                name='client_node',
                output='screen',
                arguments=['--ros-args', '--log-level', 'WARN'],
                respawn=True
            )
    network_server = Node(
                package='network_communication',
                executable='udp_server',
                name='server_node',
                output='screen',
                arguments=['--ros-args', '--log-level', 'WARN'],
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
                arguments=['/cmd_vel', '/cmd_vel/teleop', '/cmd_vel/autonomy'],
                respawn=True
            )
    twist_stamper = Node(
                package='twist_stamper',
                executable='twist_stamper',
                name='twist_stamper',
                output='screen',
                respawn=True,
                remappings=[
                    ('/cmd_vel_in', '/cmd_vel'),
                    ('/cmd_vel_out', '/base_controller/cmd_vel')
                ]
            )
    arm_drum_mux = Node(
                package='topic_tools',
                executable='mux',
                name='arm_drum_mux',
                output='screen',
                arguments=['/arm_drum_controller/commands', '/arm_drum_control/teleop', '/arm_drum_control/autonomy'],
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
    
    gstreamer = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('gstreamer'), 'gstreamer_launch.py')])
      )
    
    serial = Node(
                package='serial_node',
                executable='op_reader',
                name='serial_node',
                output='screen',
                arguments=['--ros-args', '--log-level', 'WARN'],
                parameters=[
                    {'mock_serial': 1}
                ],
                respawn=True
            )
    
    return LaunchDescription([
        teleop, 
        digdump,
        network_client,
        network_server,
        controller,
        cmd_vel_mux,
        arm_drum_mux,
        robot_state_controller,
        gstreamer,
        serial,
        
        twist_stamper,
        robot_state_publisher,
        xacro2mjcf,
        # start_mujoco,
        mujoco,
        rviz_node,
        load_controllers,
    ])

