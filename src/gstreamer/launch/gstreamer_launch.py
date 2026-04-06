import launch
import launch_ros.actions
from launch.substitutions import EnvironmentVariable

def generate_launch_description():
    control_station_ip = EnvironmentVariable("CONTROL_STATION_IP", default_value="192.168.50.220")

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='gstreamer',
            executable='webrtc_stream',
            name='stream1',
            parameters=[{
                'signaling_host': control_station_ip,
                'signaling_port': 6767,
                'video_topic': '/front_camera/image_raw'
            }]
        ),
        launch_ros.actions.Node(
            package='gstreamer',
            executable='webrtc_stream',
            name='stream2',
            parameters=[{
                'signaling_host': control_station_ip,
                'signaling_port': 6969,
                'video_topic': '/back_camera/image_raw'
            }]
        ),
    ])
