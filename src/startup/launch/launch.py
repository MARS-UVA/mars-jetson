from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
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
            arguments=['--ros-args', '--log-level', 'WARN']
        ),
        Node(
            package='digdump',
            executable='action_server',
            name='digdump',
            output='screen',
            parameters=[{
                'actuator_speed': 1,
                'dig_speed': 1,
                'dump_speed': 1,
                'drive_speed': 1,
                'dig_arm_movement_time': 5.0,
                'dump_arm_movement_time': 5.0,
                'dig_time': 5.0,
                'dump_time': 5.0,
                'move_time': 5.0,
            }]
        ),
        Node(
            package='network_communication',
            executable='net_node',
            name='network_communication',
            output='screen',
            arguments=['--ros-args', '--log-level', 'WARN']
        ),
        Node(
            package='serial_node',
            executable='op_reader',
            name='serial_node',
            output='screen',
            arguments=['--ros-args', '--log-level', 'WARN']
        ),
    ])

