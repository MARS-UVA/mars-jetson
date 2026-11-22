"""Feed RTAB-Map's retained cloud to the volatile Draco republisher."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class MapCloudRelay(Node):
    "Forward fresh clouds and replay the latest cloud when the encoder connects."

    def __init__(self):
        super().__init__('map_cloud_relay')
        self.latest_cloud = None
        self.subscriber_count = 0
        self.publisher = self.create_publisher(PointCloud2, '~/cloud', 1)
        self.subscription = self.create_subscription(
            PointCloud2,
            '/rtabmap/cloud_map',
            self.forward_cloud,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.create_timer(0.2, self.replay_to_new_subscriber)

    def forward_cloud(self, cloud):
        self.latest_cloud = cloud
        self.subscriber_count = self.publisher.get_subscription_count()
        if self.subscriber_count:
            self.publisher.publish(cloud)

    def replay_to_new_subscriber(self):
        count = self.publisher.get_subscription_count()
        if count > self.subscriber_count and self.latest_cloud is not None:
            self.publisher.publish(self.latest_cloud)
        self.subscriber_count = count


def main():
    rclpy.init()
    node = MapCloudRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
