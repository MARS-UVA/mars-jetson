#!/usr/bin/env python3

"""
A node for publishing information from a camera.

Name: webcam

Publishers
=======
webcam_image (type sensor_msgs.msg.Image): BGR8 images from a webcam (QOS: 10).
"""

from io import StringIO

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped, Point, Quaternion
from sensor_msgs.msg import Image

# OpenCV imports, use 'pip install opencv-python' to get OpenCV for Python
from apriltag_pose_estimation.core import IPHONE_13_MINI_MAIN_CAM_PARAMETERS
from apriltag_pose_estimation.localization import PoseEstimator
from apriltag_pose_estimation.localization.strategies import (MultiTagPnPEstimationStrategy,
                                                              LowestAmbiguityEstimationStrategy)
from apriltag_pose_estimation.core import load_field


FIELD_JSON = StringIO('''
{
  "fiducials": [
    {
      "id": 0,
      "rotation_vector": [
        -1.5707963267948963,
        0,
        0
      ],
      "translation_vector": [
        0,
        0,
        1.349
      ]
    },
    {
      "id": 1,
      "rotation_vector": [
        -1.5707963267948963,
        0,
        0
      ],
      "translation_vector": [
        0.2155,
        0,
        1.349
      ]
    },
    {
      "id": 2,
      "rotation_vector": [
        -1.5707963267948963,
        0,
        0
      ],
      "translation_vector": [
        0.4315,
        0,
        1.349
      ]
    },
    {
      "id": 3,
      "rotation_vector": [
        -1.5707963267948963,
        0,
        0
      ],
      "translation_vector": [
        0.6467,
        0,
        1.349
      ]
    }
  ],
  "tag_size": 0.080,
  "tag_family": "tagStandard41h12"
}
''')


VIDEO_CAPTURE_PORT = 0
FRAME_RATE_PER_SECOND = 10


class Apriltag_pose_estimate(Node):
    """
    A node that publishes information from a webcam.
    """

    def __init__(self):
        super().__init__('apriltag_estimation')  # Calling base class to assign node name
        self.subscription = self.create_subscription(  # Creating a subscription with a callback
            msg_type=Image,
            topic='webcam_image',
            callback=self.listener_callback,
            qos_profile=10  # This number which you see in the C++ publisher nodes as well is the queue size, that is how many messages to keep in the queue (subscriber queue in this case). Any message that exceeds the queue will be discarded
        )
        self.pose_estimate_publisher = self.create_publisher(
            msg_type=PoseWithCovarianceStamped,
            topic='/localization/estimated_pose',
            qos_profile=10
        )
        self.bridge = CvBridge()

        self.field = load_field(FIELD_JSON)

        self.estimator = PoseEstimator(
            strategy=MultiTagPnPEstimationStrategy(fallback_strategy=LowestAmbiguityEstimationStrategy()),
            field=self.field,
            camera_params=IPHONE_13_MINI_MAIN_CAM_PARAMETERS,
            nthreads=2,
            quad_sigma=0,
            refine_edges=1,
            decode_sharpening=0.25
        )

    def listener_callback(self, msg: Image):
        """
        Displays the given image to the local display.

        :param msg: A message containing a BGR8 image.
        """
        cv_image = self.bridge.imgmsg_to_cv2(msg)  #Convert image to OpenCV matrix
        image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        result = self.estimator.estimate_pose(image)
        if result.estimated_pose is None:
            return

        out_msg = PoseWithCovarianceStamped()

        pose_transform = result.estimated_pose.inv()

        estimated_pose_with_covariance = PoseWithCovariance()

        rotation = Quaternion()
        rotation.x, rotation.y, rotation.z, rotation.w = pose_transform.rotation.as_quat(scalar_first=False)
        estimated_pose_with_covariance.pose.orientation = rotation

        translation = Point()
        translation.x, translation.y, translation.z = pose_transform.translation
        estimated_pose_with_covariance.pose.position = translation

        estimated_pose_with_covariance.covariance = (pose_transform.error * np.eye(6)).reshape(-1)

        out_msg.pose = estimated_pose_with_covariance
        out_msg.header = msg.header
        out_msg.header.frame_id = 'world'

        self.get_logger().info('Sending pose estimate')
        self.pose_estimate_publisher.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Apriltag_pose_estimate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

