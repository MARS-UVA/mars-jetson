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
                'full_forward_magnitude': 1.0
            }]
        ),
        Node(
            package='webapp_comms',
            executable='net_node',
            name='webapp_comms',
            output='screen'
        ),
        Node(
            package='serial_ros',
            executable='op_reader',
            name='serial_ros',
            output='screen'
        )
    ])

