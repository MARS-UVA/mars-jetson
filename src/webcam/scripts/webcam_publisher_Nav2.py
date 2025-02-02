#!/usr/bin/env python3

"""
A node for publishing information from a camera.

Name: webcam

Publishers
=======
webcam_image (type sensor_msgs.msg.Image): BGR8 images from a webcam (QOS: 10).
"""

# ROS Python Libraries
#import numpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovariance
from sensor_msgs.msg import CameraInfo
from typing import Dict, List, Optional, TextIO
import sys

# OpenCV imports, use 'pip install opencv-python' to get OpenCV for Python
from cv_bridge import CvBridge
from apriltag_pose_estimation.core.camera import CameraParameters
from apriltag_pose_estimation.localization.estimator import PoseEstimator

import cv2

#sys.path.append('/home/mars_host/mars-jetson/apriltag_pose_estimation/')


VIDEO_CAPTURE_PORT = 0
FRAME_RATE_PER_SECOND = 10

DEPSTECH_CAM_PARAMETERS = CameraParameters(fx=1329.143348,
                                           fy=1326.537785,
                                           cx=945.392392,
                                           cy=521.144703,
                                           k1=-0.348650,
                                           k2=0.098710,
                                           p1=-0.000157,
                                           p2=-0.001851,
                                           k3=0.000000)
"""Camera parameters for a Depstech webcam."""

estimator = PoseEstimator(
        strategy=MultiTagPnPEstimationStrategy(fallback_strategy=LowestAmbiguityEstimationStrategy()),
        field=field,
        camera_params=DEPSTECH_CAM_PARAMETERS,
        nthreads=2,
        quad_sigma=0,
        refine_edges=1,
        decode_sharpening=0.25
    )

class WebcamPublisher(Node):
    """
    A node that publishes information from a webcam.
    """

    def __init__(self):
        super().__init__('webcam')  # Calling base class to assign node name 
        self.publisher_ = self.create_publisher(PoseWithCovariance, 'webcam_pose_with_convariance', 10)
        self.subscription = self.create_subscription(  # Creating a subscription with a callback
            Image,
            'webcam_image',
            self.listener_callback,
            10  # This number which you see in the C++ publisher nodes as well is the queue size, that is how many messages to keep in the queue (subscriber queue in this case). Any message that exceeds the queue will be discarded
        )
        #self.publisher_ = self.create_publisher(CameraInfo,'webcam_info', 10)
        # The second parameter is there because we were having issues with
        # GStreamer. CAP_V4L2 refers to Video for Linux 2, which we're using
        # as an alternative.
        self.cap = cv2.VideoCapture(VIDEO_CAPTURE_PORT, cv2.CAP_V4L2)
        #self.info_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        #self.info_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1 / FRAME_RATE_PER_SECOND,
                                       self.publish_frame)
        # self.timer2 = self.create_timer(1 / FRAME_RATE_PER_SECOND,
        #                                self.publish_camera_info)

    # def publish_frame(self):
    #     """
    #     Obtains and publishes the latest frame from the webcam to the
    #     webcam_image topic.
    #     """
    #     ret, frame = self.cap.read()
    #     if ret:
    #         img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
    #         self.publisher_.publish(img_msg)
    #         # print("height ", self.info_height)
    #         # print("width ", self.info_width)
    #         print ("cx:", CameraParameters.cx)
    #

    def publish_camera_info(self):
        ret = self.cap.read()
        if ret:
            cam_msg = self.info_width
            self.publisher_.publish(cam_msg)
            self.get_logger().info('Publishing: "%d"' % cam_msg)
            cam_msg = self.info_height
            self.publisher_.publish(cam_msg)
            self.get_logger().info('Publishing: "%d"' % cam_msg)
    def destroy_node(self):
        super().destroy_node()  # Release the video capture on node destruction
        self.cap.release()


def main(args=None):
    rclpy.init(args=args)
    node = WebcamPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

