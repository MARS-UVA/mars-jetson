import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from apriltag_msgs.msg import AprilTagDetectionArray
from robot_perception_interfaces.srv import AprilTagGate

from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
import threading


class AprilTagGateNode(Node):

    def __init__(self):
        super().__init__('AprilTagGate')
        
        self.subscriberGroup = ReentrantCallbackGroup()
        self.serviceGroup = ReentrantCallbackGroup()
        self.detectionPub = self.create_publisher(AprilTagDetectionArray, "/detections", 10)
        self.detectionsSub = self.create_subscription(AprilTagDetectionArray ,"/apriltag/detections", self.AprilTag_callback, 10, callback_group=self.subscriberGroup)
        self.AprilTagGateService = self.create_service(AprilTagGate, "/AprilTagGate", self.AprilTagService_callback, callback_group=self.serviceGroup)
        self.detectionsSub
        
        self.detectionData = AprilTagDetectionArray()
        self.TotalDetection = 0
        self.timeIndex = 0.0
        self.timeLast = 0.0
        
        self.updated = False
        self.data_lock = threading.Lock()
        self.declare_parameter("send_detections", 100)
        
        self.get_logger().info("April Tag Service Initialized")
        

    def AprilTag_callback(self, msg):
        with self.data_lock:
            self.detectionData = msg
            self.timeIndex = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1e9)
            
            if (len(msg.detections) > 0):
                self.TotalDetection = len(msg.detections)
                self.timeLast = self.timeIndex
                self.size = len(msg.detections)
                self.updated = True
                
            
        
        
    
    def AprilTagService_callback(self,request,responce):
        
        totalCall = self.get_parameter("send_detections").get_parameter_value().integer_value
        
        for i in range(1, totalCall):
            
            #waiting for subscriber to update with new tag detections
            while True:
                with self.data_lock:
                    if self.updated:
                        break
                time.sleep(0.01)
                
                
            with self.data_lock:
                snapshot = self.detectionData
                timeStamp = self.timeIndex
                self.updated = False
                
            self.detectionPub.publish(snapshot)
            totalSnapshots = len(snapshot.detections)
            self.get_logger().info(f"Total AprilTag Detections: {self.TotalDetection}")
            
            #should log all tag data for each detection
            for detection in range(totalSnapshots):
                family = snapshot.detections[detection].family
                id = snapshot.detections[detection].id
                centre = snapshot.detections[detection].centre
                cornerdata = snapshot.detections[detection].corners
                
                position = "x: " + str(centre.x) + "y: " + str(centre.y)
                corners = f"1:({cornerdata[0].x}, {cornerdata[0].y}) 2:({cornerdata[1].x}, {cornerdata[1].y}) 3:({cornerdata[2].x}, {cornerdata[2].y}) 4:({cornerdata[3].x}, {cornerdata[3].y})"
                alltags = "Family: " + str(family) + "ID: " + str(id) + " at " + str(timeStamp) + " seconds since startup | position: " + position + "| corners: " + corners
                self.get_logger().info(alltags)
                responce.data += alltags
            
            responce.success = True
        
        
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