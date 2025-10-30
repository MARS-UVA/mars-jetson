#!/usr/bin/env python3

"""
A node for publishing information from a camera.

Name: webcam

Publishers
=======
webcam_image (type sensor_msgs.msg.Image): BGR8 images from a webcam (QOS: 10).
"""

# ROS Python Libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# OpenCV imports, use 'pip install opencv-python' to get OpenCV for Python
from cv_bridge import CvBridge
import cv2


VIDEO_CAPTURE_PORT = 0
FRAME_RATE_PER_SECOND = 10


class WebcamPublisher(Node):
    """
    A node that publishes information from a webcam.
    """

    def __init__(self):
        super().__init__('webcam')  # Calling base class to assign node name 
        self.publisher_ = self.create_publisher(Image, 'webcam_image', 10)
        # The second parameter is there because we were having issues with
        # GStreamer. CAP_V4L2 refers to Video for Linux 2, which we're using
        # as an alternative.
        self.cap = cv2.VideoCapture(VIDEO_CAPTURE_PORT, cv2.CAP_V4L2)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1 / FRAME_RATE_PER_SECOND,
                                       self.publish_frame)

    def publish_frame(self):
        """
        Obtains and publishes the latest frame from the webcam to the
        webcam_image topic.
        """
        ret, frame = self.cap.read()
        if ret:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(img_msg)

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

