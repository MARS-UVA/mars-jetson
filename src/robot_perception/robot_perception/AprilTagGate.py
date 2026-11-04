import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped
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
        self.timeIndex = 0.0
        self.timeLast = 0.0
        self.size = 0
        

    def AprilTag_callback(self, msg):
        self.detectionData = msg
        self.timeIndex = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1e9)
        if (len(msg.detections) > 0) and (len(msg.detections) > self.size):
            self.TotalDetection += 1
            self.size = len(msg.detections)
        if self.TotalDetection > 1:
             self.timeLast = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1e9)
        
        
        
    
    def AprilTagService_callback(self,request,responce):
        if (self.timeIndex - self.timeLast > 0.1) and (self.timeIndex - self.timeLast <= 0.5) and (self.TotalDetection > 1):
                self.detectionPub.publish(self.detectionData)
                family = self.detectionData.detections[self.TotalDetection -1].family
                id = self.detectionData.detections[self.TotalDetection -1].id
                pose = self.detectionData.detections[self.TotalDetection -1].pose
                position = "x: " + str(pose.pose.position.x) + "y: " + str(pose.pose.position.y) + "z: " + str(pose.pose.position.z)
                orientation = "x: " + str(pose.pose.orientation.x) + "y: " + str(pose.pose.orientation.y) + "z: " + str(pose.pose.orientation.z) + "w: " + str(pose.pose.orientation.w)
                responce.data = "Family: " + str(family) + "ID: " + id + " at " + str(self.timeIndex) + " seconds since startup | position: " + position + "| Orientation: " +  orientation
                responce.success = True
                self.detectionData = AprilTagDetectionArray()
        else:
            responce.success = False
            responce.data = "No Detections"
        
        return responce
        
        
        
        


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