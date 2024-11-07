#!/usr/bin/env python3

# ROS Python Libraries
from pupil_apriltags import Detector
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# OpenCV imports, use 'pip install opencv-python' to get OpenCV for Python
from cv_bridge import CvBridge
import cv2
import numpy as np

# Webcam Subscriber node which inherits the Node parent class from rclpy
class WebcamSubscriber(Node):
    def __init__(self) -> None:
        super().__init__("webcam_subscriber") # Calling parents class to assign node name 
        self.get_logger().info("Webcam subscriber node init")
        self.subscription = self.create_subscription(  #Creating a subscription with a callback
            Image,
            'webcam_image',
            self.listener_callback,
            10  # This number which you see in the C++ publisher nodes as well is the queue size, that is how many messages to keep in the queue (subscriber queue in this case). Any message that exceeds the queue will be dicarded
        )
        self.bridge = CvBridge()  #Instantiating a CVBridge instance

    #Callback function called when a message is ready to be processed from the subscriber queue
    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')  #Convert image to OpenCV matrix

        #cv2.imshow("Image", cv_image)
        #cv2.waitKey(1)

        cv_image = cv_image.mean(axis=2).astype(np.uint8)
        print(cv_image.shape)


        at_detector = Detector(
           families="tagStandard41h12",
           nthreads=1,
           quad_decimate=1.0,
           quad_sigma=0.0,
           refine_edges=1,
           decode_sharpening=0.25,
           debug=0
         )
        print(at_detector.detect(cv_image))
    

# Main method is the first point of entry which instantiates an instance of the WebcamSubscriber class (a child of the Node class)
def main(args=None):
    rclpy.init()
    node = WebcamSubscriber()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()







""" import cv2
import numpy as np
from apriltag import apriltag

imagepath = 'test.jpg'
image = cv2.imread(imagepath, cv2.IMREAD_GRAYSCALE)
detector = apriltag("tagStandard41h12")

detections = detector.detect(image)

print(detections) """