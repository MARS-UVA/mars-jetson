import sys

from robot_perception_interfaces.srv import AprilTagGate
import rclpy
from rclpy.node import Node


class AprilTagClient(Node):

    def __init__(self):
        super().__init__('AprilTagClient')
        self.AprilTagClient = self.create_client(AprilTagGate, '/AprilTagGate')
        while not self.AprilTagClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = None



def main():
    rclpy.init()

    AprilTagClientNode = AprilTagClient()
    future = AprilTagClientNode.send_request()
    rclpy.spin_until_future_complete(AprilTagClientNode, future)
    response = future.result()

    AprilTagClientNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()