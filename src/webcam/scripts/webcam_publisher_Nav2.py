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
# from geometry_msgs.msg import PoseWithCovariance
# from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
import rclpy

# OpenCV imports, use 'pip install opencv-python' to get OpenCV for Python
from cv_bridge import CvBridge
from apriltag_pose_estimation.core import CameraParameters
from apriltag_pose_estimation.localization import PoseEstimator
from apriltag_pose_estimation.localization.strategies import (MultiTagPnPEstimationStrategy, 
                                                              LowestAmbiguityEstimationStrategy)

from apriltag_pose_estimation.core import load_field

with open('/home/mars_host/mars-jetson/src/webcam/scripts/resources/field.json', mode='r') as f:
    field = load_field(f)

import cv2


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

class Apriltag_pose_estimate(Node):
    """
    A node that publishes information from a webcam.
    """

    def __init__(self):
        super().__init__('apriltag_estimation')  # Calling base class to assign node name 
        self.subscription = self.create_subscription(  # Creating a subscription with a callback
            Image,
            'webcam_image',
            self.listener_callback,
            10  # This number which you see in the C++ publisher nodes as well is the queue size, that is how many messages to keep in the queue (subscriber queue in this case). Any message that exceeds the queue will be discarded
        )
        self.bridge = CvBridge()

    def listener_callback(self, msg: String):
        """
        Displays the given image to the local display.
            
        :param msg: A message containing a BGR8 image.
        """
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')  #Convert image to OpenCV matrix
        image=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        result = estimator.estimate_pose(image)
        print(result.estimated_pose)


def main(args=None):
    rclpy.init(args=args)
    node = Apriltag_pose_estimate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

