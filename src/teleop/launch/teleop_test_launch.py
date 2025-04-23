import launch_ros.actions
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='teleop',
            namespace='',
            executable='teleop',
            name='teleop',
            parameters=[
                {'linear_axis': 'left_y'},
                {'turn_axis': 'right_x_inverted'},
                {'full_forward_magnitude': 0.6},
                {'deadband': 0.1}
            ]
        ),
    ])