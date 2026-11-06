import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from apriltag_msgs.msg import AprilTagDetectionArray
from robot_perception_interfaces.srv import AprilTagGate

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import asyncio


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
        
        self.updated = asyncio.Event()
        self.declare_parameter("send_detections", 100)
        
        self.get_logger().info("April Tag Service Initialized")
        

    def AprilTag_callback(self, msg):
        self.detectionData = msg
        self.timeIndex = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1e9)
        
        if (len(msg.detections) > 0) and (len(msg.detections) > self.size):
            self.TotalDetection += 1
            self.size = len(msg.detections)
            self.updated.set()
        else:
            self.updated = False
            
        if self.TotalDetection > 0:
             self.timeLast = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1e9)
        
        
        
    
    async def AprilTagService_callback(self,request,responce):
        
        totalCall = self.get_parameter("send_detections").get_parameter_value().integer_value
        
        for i in range(totalCall):
            await self.updated.wait()
            self.detectionPub.publish(self.detectionData)
            
            #The array index might need to be zero since detection data is cleared at the end
            #Figure out when testing if an error is thrown
            family = self.detectionData.detections[self.TotalDetection -1].family
            id = self.detectionData.detections[self.TotalDetection -1].id
            pose = self.detectionData.detections[self.TotalDetection -1].pose
            
            position = "x: " + str(pose.pose.position.x) + "y: " + str(pose.pose.position.y) + "z: " + str(pose.pose.position.z)
            orientation = "x: " + str(pose.pose.orientation.x) + "y: " + str(pose.pose.orientation.y) + "z: " + str(pose.pose.orientation.z) + "w: " + str(pose.pose.orientation.w)
            
            responce.data = "Family: " + str(family) + "ID: " + id + " at " + str(self.timeIndex) + " seconds since startup | position: " + position + "| Orientation: " +  orientation
            
            self.get_logger().info(responce.data)
            responce.success = True
            self.detectionData = AprilTagDetectionArray()
            self.size = len(self.detectionData.detections)
            self.updated.clear()
        
        self.get_logger().info(f"Total AprilTag Detections: {self.TotalDetection}")
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