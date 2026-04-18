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
                    'actuator_speed': 15,
                    'dig_speed': 50,
                    'dump_speed': 50,
                    'drive_speed': 1,
                    'dig_arm_movement_time': 12.5,
                    'dump_arm_movement_time': 0.0,
                    'dig_time': 15.0,
                    'dump_time': 15.0,
                    'move_time': 2.5,
                }],
                respawn=True
            )
    network = Node(
                package='network_communication',
                executable='net_node',
                name='network_communication',
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
        network,
        serial,
        cameras,
    ])

