import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='gstreamer',
            executable='webrtc_stream',
            name='stream1',
            parameters=[{
                'signaling_url': 'ws://192.168.50.101:6767',
                'video_topic': '/camera1/image_raw'
            }]
        ),
        launch_ros.actions.Node(
            package='gstreamer',
            executable='webrtc_stream',
            name='stream2',
            parameters=[{
                'signaling_url': 'ws://192.168.50.101:6969',
                'video_topic': '/camera2/image_raw'
            }]
        ),
    ])