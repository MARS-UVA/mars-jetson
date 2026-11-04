import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('startup'),
        'config',
        'oakd_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='depthai_ros_driver_v3',
            executable='driver_node',
            name='rgbd_blue',
            parameters=[config_file],
            remappings=[('/rgbd_blue/rgbd/points', '/rgbd_blue/points')]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='rgbd_blue_static_tf',
            arguments=[
                '--x',
                '0.0',
                '--y',
                '-0.125',
                '--z',
                '0.0',
                '--roll',
                '-0.261799',
                '--pitch',
                '0.0',
                '--yaw',
                '0.0',
                '--frame-id',
                'front_camera_color_optical_frame',
                '--child-frame-id',
                'rgbd_blue_rgb_camera_optical_frame',
            ],
        ),
    ])
