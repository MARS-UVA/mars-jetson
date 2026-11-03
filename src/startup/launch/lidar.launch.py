from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    point_lio = FindPackageShare('point_lio')

    # Run unitree lidar
    unitree_lidar = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        parameters= [
                {'publish_tf': False},
                {'initialize_type': 2},
                {'work_mode': 0},
                {'use_system_timestamp': True},
                {'range_min': 0.0},
                {'range_max': 100.0},
                {'cloud_scan_num': 18},

                {'serial_port': '/dev/ttyACM0'},
                {'baudrate': 4000000},

                {'lidar_port': 6101},
                {'lidar_ip': '192.168.1.62'},
                {'local_port': 6201},
                {'local_ip': '192.168.1.2'},
                
                {'cloud_frame': "unilidar_lidar"},
                {'cloud_topic': "unilidar/cloud"},
                {'imu_frame': "unilidar_imu"},
                {'imu_topic': "unilidar/imu"},
                ]
    )

    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_to_base_tf',
        arguments=[
            '--x',
            '0.0',
            '--y',
            '0.0',
            '--z',
            '0.5',  # base_link is 0.5m below the lidar optical center
            '--roll',
            '3.14159', # base_link is upside down relative to the lidar optical center
            '--pitch',
            '0.0',
            '--yaw',
            '1.048',  # base_link is rotated 60 degrees relative to the lidar optical center
            '--frame-id',
            'unilidar_lidar',
            '--child-frame-id',
            'base_link',
        ],
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([point_lio, 'launch', 'mapping_unilidar_l2.launch.py'])),
            launch_arguments={
                'rviz': 'false',
                'use_sim_time': 'false',
            }.items()
        ),
        unitree_lidar,
        static_transform_publisher
    ])
