from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
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
    
    # The OV9281 sensors are monochrome, so publish mono8 by default: v4l2_camera
    # converts the YUYV frames with cv_bridge (YUV2GRAY_YUY2), which halves the
    # raw image size and gives the H.264 encoder exactly neutral chroma.
    # color_cameras:=true keeps the camera's native YUYV output instead.
    OUTPUT_ENCODING = PythonExpression([
        "'yuv422_yuy2' if '", LaunchConfiguration('color_cameras'),
        "'.lower() == 'true' else 'mono8'",
    ])
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'color_cameras', default_value='false',
            description='Publish the native YUYV (color) output instead of mono8',
        ),
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
               'output_encoding': OUTPUT_ENCODING,
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
               'output_encoding': OUTPUT_ENCODING,
               **FFMPEG_ENCODER,
            #    'camera_info_url': 'file:///home/ericzn248/.ros/camera_info/narrow_stereo.yaml',
           }]
        ),
    ])
