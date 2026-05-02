import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
                    'actuator_speed_aerial': 127,
                    'actuator_speed_ground': 75,
                    'dig_speed': 100,
                    'dump_speed': 127,
                    'drive_speed': 30,
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
        cameras,
    ])

