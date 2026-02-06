from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            namespace='awareness/arducam1',
            executable='v4l2_camera_node',
            name='webcam',
            parameters=[{
                'video_device': '/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0',
                'image_size': [1280, 720],
                'output_encoding': 'yuv422_yuy2',
                # 'camera_info_url': 'file:///home/ericzn248/.ros/camera_info/narrow_stereo.yaml',
            }]
        ),
    ])