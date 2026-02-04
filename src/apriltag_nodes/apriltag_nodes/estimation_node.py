#!/usr/bin/env python3

from io import StringIO

import cv2
import numpy as np
import rclpy
import tf2_ros
import tf2_py as tf2
from scipy.spatial.transform import Rotation
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped, Point, Quaternion
from sensor_msgs.msg import Image

# OpenCV imports, use 'pip install opencv-python' to get OpenCV for Python
from apriltag_pose_estimation.core import IPHONE_13_MINI_MAIN_CAM_PARAMETERS, Transform, load_field
from apriltag_pose_estimation.localization import PoseEstimator
from apriltag_pose_estimation.localization.strategies import (MultiTagPnPEstimationStrategy,
                                                              LowestAmbiguityEstimationStrategy)


VIDEO_CAPTURE_PORT = 0
FRAME_RATE_PER_SECOND = 10


# FIELD_JSON = StringIO('''
# {
#   "fiducials": [
#     {
#       "id": 0,
#       "rotation_vector": [
#         -1.5707963267948963,
#         0,
#         0
#       ],
#       "translation_vector": [
#         0.149,
#         0,
#         0.214
#       ]
#     },
#     {
#       "id": 1,
#       "rotation_vector": [
#         -1.2091995761561447,
#         1.209199576156145,
#         -1.209199576156145
#       ],
#       "translation_vector": [
#         0,
#         0.145,
#         0.214
#       ]
#     },
#     {
#       "id": 2,
#       "rotation_vector": [
#         -1.2091995761561447,
#         1.209199576156145,
#         -1.209199576156145
#       ],
#       "translation_vector": [
#         0,
#         0.426,
#         0.214
#       ]
#     },
#     {
#       "id": 3,
#       "rotation_vector": [
#         -1.2091995761561447,
#         1.209199576156145,
#         -1.209199576156145
#       ],
#       "translation_vector": [
#         0,
#         0.701,
#         0.219
#       ]
#     }
#   ],
#   "tag_size": 0.135,
#   "tag_family": "tagStandard41h12"
# }
# ''')


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


class AprilTagPoseEstimationNode(Node):
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
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

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

        camera_pose_transform = result.estimated_pose.inv()

        try:
            robot_to_camera_transform_msg = self.tf_buffer.lookup_transform(target_frame='camera_optical', source_frame='base_link', time=Time())
        except tf2.LookupException:
            self.get_logger().error('Cannot find robot-to-camera transform')
            return
        robot_to_camera_transform = Transform.make(
            rotation=Rotation.from_quat([robot_to_camera_transform_msg.transform.rotation.x,
                                         robot_to_camera_transform_msg.transform.rotation.y,
                                         robot_to_camera_transform_msg.transform.rotation.z,
                                         robot_to_camera_transform_msg.transform.rotation.w],
                                        scalar_first=False),
            translation=np.array([robot_to_camera_transform_msg.transform.translation.x,
                                  robot_to_camera_transform_msg.transform.translation.y,
                                  robot_to_camera_transform_msg.transform.translation.z]),
            input_space='base_link',
            output_space='camera_optical'
        )
        robot_pose_transform = camera_pose_transform @ robot_to_camera_transform

        estimated_pose_with_covariance = PoseWithCovariance()

        rotation = Quaternion()
        rotation.x, rotation.y, rotation.z, rotation.w = robot_pose_transform.rotation.as_quat(scalar_first=False)
        estimated_pose_with_covariance.pose.orientation = rotation

        translation = Point()
        translation.x, translation.y, translation.z = robot_pose_transform.translation
        estimated_pose_with_covariance.pose.position = translation

        estimated_pose_with_covariance.covariance = (((6 * camera_pose_transform.error) ** 2) * np.eye(6)).reshape(-1)

        out_msg.pose = estimated_pose_with_covariance
        out_msg.header = msg.header
        out_msg.header.frame_id = robot_pose_transform.output_space

        self.get_logger().debug(f'Sending pose estimate {result.estimated_pose}')
        self.pose_estimate_publisher.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagPoseEstimationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
