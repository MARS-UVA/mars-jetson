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
            package='webapp_comms',
            executable='net_node',
            name='webapp_comms',
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
        Node(
            package='webcam',
            executable='webcam',
            name='webcam',
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO']
        )
    ])

