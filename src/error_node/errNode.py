#!/usr/bin/env python3

# ROS Python Libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# Webcam Subscriber node which inherits the Node parent class from rclpy
class ErrorDetector(Node):
    def __init__(self) -> None:
        super().__init__("ErrorDetector") # Calling parents class to assign node name 
        self.subscription = self.create_subscription(  #Creating a subscription with a callback
            str,
            'error_publisher',
            self.listener_callback,
            10  # This number which you see in the C++ publisher nodes as well is the queue size, that is how many messages to keep in the queue (subscriber queue in this case). Any message that exceeds the queue will be dicarded
        )

    #Callback function called when a message is ready to be processed from the subscriber queue
    def listener_callback(self, msg):
        # Publish to control station that an error has been detected
        
        # Stop all robot operation
    
# Main method is the first point of entry which instantiates an instance of the WebcamSubscriber class (a child of the Node class)
def main(args=None):
    rclpy.init()
    node = WebcamSubscriber()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
