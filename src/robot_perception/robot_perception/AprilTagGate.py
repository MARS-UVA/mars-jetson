import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from apriltag_msgs.msg import AprilTagDetectionArray
from robot_perception_interfaces.srv import AprilTagGate


class AprilTagGateNode(Node):

    def __init__(self):
        super().__init__('AprilTagGate')
        self.detectionPub = self.create_publisher(AprilTagDetectionArray, "/detection", 10)
        self.detectionsSub = self.create_subscription(AprilTagDetectionArray ,"/detections", self.AprilTag_callback, 10)
        self.AprilTagGateService = self.create_service(AprilTagGate, "/AprilTagGate", self.AprilTagService_callback)
        self.detectionsSub
        self.detectionData = AprilTagDetectionArray()
        self.TotalDetection = 0
        self.get_logger().info("April Tag Service Initialized")
        

    def AprilTag_callback(self, msg):
        self.detectionData = msg
        
        
    
    def AprilTagService_callback(self, request, responce):
        if self.TotalDetection < 1:
            self.detectionPub.publish(self.detectionData)
            dataResponce = String()
            dataResponce.data = str(self.detectionData[self.TotalDetection].tag)
            responce.data = dataResponce.data #Figure out what data is supposed to be
            responce.success = True
            self.TotalDetection += 1
        else:
            responce.success = False
        
        
        
        


def main(args=None):
    rclpy.init(args=args)

    node = AprilTagGateNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()