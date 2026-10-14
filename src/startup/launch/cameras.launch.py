from launch import LaunchDescription
from launch_ros.actions import Node

# H.264 encoding for the dashboard (ffmpeg_image_transport). The plugin adds an
# image_raw/ffmpeg topic next to image_raw and only encodes while something
# subscribes to it, so an unviewed camera costs no encoding CPU. The Jetson has
# no hardware encoder: libx264 runs on the CPU, so keep the preset fast.
# Tune bit_rate, qmax and frame rate on a real driving run.
FFMPEG_ENCODER = {
    'image_raw.ffmpeg.encoder': 'libx264',
    # zerolatency: no lookahead, no B-frames. baseline: decodable by every browser.
    'image_raw.ffmpeg.encoder_av_options': 'preset:superfast,tune:zerolatency,profile:baseline',
    'image_raw.ffmpeg.gop_size': 30,       # keyframe every 2 s at 15 fps
    'image_raw.ffmpeg.bit_rate': 300000,   # bits/s; plugin default is ~8 Mbps
    'image_raw.ffmpeg.qmax': 40,           # plugin default of 10 ignores bit_rate
}

def generate_launch_description():
    return LaunchDescription([
        Node(
           package='v4l2_camera',
           namespace='front_camera',
           executable='v4l2_camera_node',
           parameters=[{
               # 'video_device': '/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0',
               # 'video_device': '/dev/video0',
               'video_device': '/dev/v4l/by-path/platform-3610000.usb-usb-0:2.2:1.0-video-index0',
               'image_size': [480, 320],
               'time_per_frame': [1, 15],  # 15 fps
               # If the encoder rejects yuv422_yuy2, use 'mono8' (the cameras are monochrome).
               'output_encoding': 'yuv422_yuy2',
               **FFMPEG_ENCODER,

            #    'camera_info_url': 'file:///home/ericzn248/.ros/camera_info/narrow_stereo.yaml', fill out later
           }]
        ),
        Node(
           package='v4l2_camera',
           namespace='rear_camera',
           executable='v4l2_camera_node',
           parameters=[{
               # 'video_device': '/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index1',
               # 'video_device': '/dev/video2',
               'video_device': '/dev/v4l/by-path/platform-3610000.usb-usb-0:2.4:1.0-video-index0',
               'image_size': [480, 320],
               # 'image_size': [1280, 720],
               'time_per_frame': [1, 15],  # 15 fps
               'output_encoding': 'yuv422_yuy2',
               **FFMPEG_ENCODER,
            #    'camera_info_url': 'file:///home/ericzn248/.ros/camera_info/narrow_stereo.yaml',
           }]
        ),
    ])
