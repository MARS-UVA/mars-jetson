#!/usr/bin/env python3

"""
A node that displays the stream from the webcam on the local display.

Name: display

Subscribers
============
webcam_image (type sensor_msgs.msg.Image): An BGR8 image from the webcam (QOS: 10).
"""

# ROS Python Libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# OpenCV imports, use 'pip install opencv-python' to get OpenCV for Python
from cv_bridge import CvBridge
import cv2


# Webcam Subscriber node which inherits the Node parent class from rclpy
class WebcamSubscriber(Node):
    """
    A node which publishes images from webcam_topic to the local display.
    """

    def __init__(self):
        super().__init__('webcam_subscriber')  # Calling parents class to assign node name 
        self.get_logger().info('Webcam subscriber node init')
        self.subscription = self.create_subscription(  # Creating a subscription with a callback
            Image,
            'webcam_image',
            self.listener_callback,
            10  # This number which you see in the C++ publisher nodes as well is the queue size, that is how many messages to keep in the queue (subscriber queue in this case). Any message that exceeds the queue will be discarded
        )
        self.bridge = CvBridge()  #Instantiating a CVBridge instance

    def listener_callback(self, msg: Image):
        """
        Displays the given image to the local display.
        
        :param msg: A message containing a BGR8 image.
        """
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')  #Convert image to OpenCV matrix

        cv2.imshow('Image', cv_image)
        cv2.waitKey(1)
    

# Main method is the first point of entry which instantiates an instance of the WebcamSubscriber class (a child of the Node class)
def main(args=None):
    rclpy.init()
    node = WebcamSubscriber()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

