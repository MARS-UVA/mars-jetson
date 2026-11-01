import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from apriltag_msgs.msg import AprilTagDetectionArray
from robot_perception_interfaces.srv import AprilTagGate


class AprilTagGateNode(Node):

    def __init__(self):
        super().__init__('AprilTagGate')
        self.detectionPub = self.create_publisher(AprilTagDetectionArray, "/detections", 10)
        self.detectionsSub = self.create_subscription(AprilTagDetectionArray ,"/apriltag/detections", self.AprilTag_callback, 10)
        self.AprilTagGateService = self.create_service(AprilTagGate, "/AprilTagGate", self.AprilTagService_callback)
        self.detectionsSub
        self.detectionData = AprilTagDetectionArray()
        self.TotalDetection = 0
        self.get_logger().info("April Tag Service Initialized")
        self.timeIndex = 0
        self.timeLast = 0
        self.timer = self.create_timer(0.1, self.timerCallback)
        
    def timerCallback(self):
        self.timeIndex += 0.1
        

    def AprilTag_callback(self, msg):
        self.detectionData = msg
        
        
    
    def AprilTagService_callback(self,request,responce):
        if self.TotalDetection < 1 and (self.timeIndex - self.timeLast > 0.1):
            self.timeLast = self.timeIndex
            self.detectionPub.publish(self.detectionData)
            dataResponce = String()
            dataResponce.data = str(self.detectionData.detections[self.TotalDetection].family)
            responce.data = dataResponce.data + " at " + self.timeIndex + " seconds since startup"  #Figure out what data is supposed to be
            responce.success = True
            self.detectionData = AprilTagDetectionArray()
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