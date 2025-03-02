#!/usr/bin/env python3

# ROS Python Libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# OpenCV imports, use 'pip install opencv-python' to get OpenCV for Python
from cv_bridge import CvBridge
import cv2

#to use C library
import ctypes

#load shared library
lib = ctypes.CDLL('client_udp.c')

# Webcam Subscriber node which inherits the Node parent class from rclpy
class WebcamSubscriber(Node):
    def __init__(self) -> None:
        super().__init__("webcam_subscriber") # Calling parents class to assign node name 
        self.get_logger().info("Webcam subscriber node init")
        self.subscription = self.create_subscription(  #Creating a subscription with a callback
            Image,
            'webcam_topic',
            self.listener_callback,
            10  # This number which you see in the C++ publisher nodes as well is the queue size, that is how many messages to keep in the queue (subscriber queue in this case). Any message that exceeds the queue will be dicarded
        )
        self.bridge = CvBridge()  #Instantiating a CVBridge instance

    #Callback function called when a message is ready to be processed from the subscriber queue
    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')  #Convert image to OpenCV matrix

        # Encode the image to a byte array
        success, encoded_image = cv2.imencode('.jpg', cv_image)
        if success:
            byte_array = encoded_image.tobytes()  # Convert to bytes

        lib.client("127.0.0.1", byte_array)

        # Display the image (Optional)
        cv2.imshow("Image", cv_image)
        cv2.waitKey(1)


        cv2.imshow("Image", cv_image)
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