import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from apriltag_msgs.msg import AprilTagDetectionArray
from robot_perception_interfaces.srv import AprilTagGate

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class AprilTagGateNode(Node):

    def __init__(self):
        super().__init__('AprilTagGate')
        
        self.subscriberGroup = ReentrantCallbackGroup()
        self.detectionPub = self.create_publisher(AprilTagDetectionArray, "/detections", 10, callback_group=self.subscriberGroup)
        self.detectionsSub = self.create_subscription(AprilTagDetectionArray ,"/apriltag/detections", self.AprilTag_callback, 10)
        self.AprilTagGateService = self.create_service(AprilTagGate, "/AprilTagGate", self.AprilTagService_callback)
        self.detectionsSub
        
        self.detectionData = AprilTagDetectionArray()
        self.TotalDetection = 0
        self.timeIndex = 0.0
        self.timeLast = 0.0
        self.size = 0
        
        self.updated = False
        self.declare_parameter("send_detections", 100)
        
        self.get_logger().info("April Tag Service Initialized")
        

    def AprilTag_callback(self, msg):
        self.detectionData = msg
        self.timeIndex = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1e9)
        
        if (len(msg.detections) > 0) and (len(msg.detections) > self.size):
            self.TotalDetection += len(msg.detections)
            self.size = len(msg.detections)
            self.updated = True
            
        if self.TotalDetection > 0:
             self.timeLast = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1e9)
        
        
        
    
    async def AprilTagService_callback(self,request,responce):
        
        totalCall = self.get_parameter("send_detections").get_parameter_value().integer_value
        
        for i in range(1, totalCall):
            
            #waiting for subscriber to update with new tag detections
            while self.updated == False:
                self.get_logger().info("waiting for new detection")
            self.detectionPub.publish(self.detectionData)
            
            self.get_logger().info(f"Total AprilTag Detections: {self.TotalDetection}")
            
            #should log all tag data for each detection
            for detection in range(self.totalDetection):
                family = self.detectionData.detections[detection].family
                id = self.detectionData.detections[detection].id
                pose = self.detectionData.detections[detection].pose
                
                position = "x: " + str(pose.pose.position.x) + "y: " + str(pose.pose.position.y) + "z: " + str(pose.pose.position.z)
                orientation = "x: " + str(pose.pose.orientation.x) + "y: " + str(pose.pose.orientation.y) + "z: " + str(pose.pose.orientation.z) + "w: " + str(pose.pose.orientation.w)
                
                responce.data = "Family: " + str(family) + "ID: " + id + " at " + str(self.timeIndex) + " seconds since startup | position: " + position + "| Orientation: " +  orientation
                self.get_logger().info(responce.data)
            
            responce.success = True
            
            #clears data of previous detection
            self.detectionData = AprilTagDetectionArray()
            self.size = len(self.detectionData.detections)
            self.totalDetection = 0
            self.updated.clear()
        
        
        return responce
        
        
        
        


def main(args=None):
    rclpy.init(args=args)

    node = AprilTagGateNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()