from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
      
   arducam_info_path = (Path(get_package_share_directory('startup')) / 'resource' / 'arducam1.yaml').as_uri()

   return LaunchDescription([
        Node(
           package='v4l2_camera',
           namespace='front_camera',
           executable='v4l2_camera_node',
           parameters=[{
               # 'video_device': '/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0',
               'video_device': '/dev/video0',
               'image_size': [480, 320],
               'output_encoding': 'yuv422_yuy2',
               'camera_info_url': arducam_info_path
            #    'camera_info_url': 'file:///home/ericzn248/.ros/camera_info/narrow_stereo.yaml', fill out later
           }]
        ),
        Node(
           package='v4l2_camera',
           namespace='back_camera',
           executable='v4l2_camera_node',
           parameters=[{
               # 'video_device': '/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index1',
               'video_device': '/dev/video2',
               'image_size': [480, 320],
               # 'image_size': [1280, 720],
               'output_encoding': 'yuv422_yuy2',
               'camera_info_url': arducam_info_path
            #    'camera_info_url': 'file:///home/ericzn248/.ros/camera_info/narrow_stereo.yaml',
           }]
        ),
    ])

