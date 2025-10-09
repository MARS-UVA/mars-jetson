#!/usr/bin/env python3

# ROS Python Libraries
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

# Webcam Subscriber node which inherits the Node parent class from rclpy
class HeartbeatDetector(Node):
    def __init__(self) -> None:

        self.nodes_to_monitor = ["teleop", "serialROS"] # List of nodes to monitor
        self.last_heartbeat_times = {}

        for node in self.nodes_to_monitor:
            topic = f"health/{node}"
            self.last_heartbeat_times[node] = self.get_clock().now()
            self.create_subscription(
                Bool, topic, lambda msg, n=node: self.heartbeat_callback(msg, n), 
                10)
        self.timeout = Duration(seconds=2)
        self.timer = self.create_timer(1.0, self.check_heartbeats)
        

    def heartbeat_callback(self, msg, node):
        if msg.data:
            self.last_heartbeat_time[node] = self.get_clock().now()   
            self.get_logger().info(f"{self.node} Alive")

    def check_heartbeats(self):
        now = self.get_clock().now()
        for node, last_time in self.last_heartbeat_times.items():
            if now - last_time > self.timeout:
                self.get_logger.warn(f"{self.node} Node Unresponsive")
    
# Main method is the first point of entry which instantiates an instance of the WebcamSubscriber class (a child of the Node class)
def main(args=None):
    rclpy.init()
    node = WebcamSubscriber()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
