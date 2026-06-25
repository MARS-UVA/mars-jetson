import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
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
                    'actuator_speed_aerial': 1.0,
                    'actuator_speed_ground': 0.67,
                    'dig_speed': 0.75,
                    'dump_speed': 1.0,
                    'drive_speed': 0.25,
                    'dig_time': 3.0,
                    'dump_time': 6.0,
                    'move_time': 5.0,
                    'actuator_extend_length_aerial': 0.69,
                    'actuator_extend_length_ground': 0.85,
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
    serial = Node(
                package='serial_node',
                executable='op_reader',
                name='serial_node',
                output='screen',
                arguments=['--ros-args', '--log-level', 'WARN'],
                parameters=[
                    {'mock_serial': EnvironmentVariable('MOCK_SERIAL', default_value='0')}
                ],
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
    arm_drum_mux = Node(
                package='topic_tools',
                executable='mux',
                name='arm_drum_mux',
                output='screen',
                arguments=['/arm_drum_control', '/arm_drum_control/teleop', '/arm_drum_control/autonomy'],
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
    
    cameras = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('startup'), 'launch'),
         '/cameras.launch.py'])
      )

    return LaunchDescription([
        teleop, 
        digdump,
        network_client,
        network_server,
        serial,
        controller,
        cmd_vel_mux,
        arm_drum_mux,
        robot_state_controller,
        cameras,
    ])

