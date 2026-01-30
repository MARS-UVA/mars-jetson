#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')

        # Publisher 1: original circular motion
        self.pub_cov = self.create_publisher(
            PoseWithCovarianceStamped,
            '/pose_with_cov',
            10
        )

        # Publisher 2: linear motion for estimated_pose
        self.pub_estimated = self.create_publisher(
            PoseWithCovarianceStamped,
            '/localization/estimated_pose',
            10
        )

        # Timer for 10 Hz
        self.timer = self.create_timer(0.1, self.publish_pose)
        self.t = 0.0

    def publish_pose(self):
        now = self.get_clock().now().to_msg()

        # --------------------------
        # Publisher 1: /pose_with_cov
        # --------------------------
        msg_cov = PoseWithCovarianceStamped()
        msg_cov.header.stamp = now
        msg_cov.header.frame_id = 'odom'

        # Circular motion
        msg_cov.pose.pose.position.x = math.cos(self.t)
        msg_cov.pose.pose.position.y = math.sin(self.t)
        msg_cov.pose.pose.position.z = 0.0

        msg_cov.pose.pose.orientation.x = 0.0
        msg_cov.pose.pose.orientation.y = 0.0
        msg_cov.pose.pose.orientation.z = 0.0
        msg_cov.pose.pose.orientation.w = 1.0

        # Reasonable covariance
        msg_cov.pose.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]

        self.pub_cov.publish(msg_cov)

        # --------------------------
        # Publisher 2: /localization/estimated_pose
        # --------------------------
        msg_est = PoseWithCovarianceStamped()
        msg_est.header.stamp = now
        msg_est.header.frame_id = 'odom'

        # Linear motion along x
        msg_est.pose.pose.position.x = self.t  # moves forward
        msg_est.pose.pose.position.y = 0.0
        msg_est.pose.pose.position.z = 0.0

        msg_est.pose.pose.orientation.x = 0.0
        msg_est.pose.pose.orientation.y = 0.0
        msg_est.pose.pose.orientation.z = 0.0
        msg_est.pose.pose.orientation.w = 1.0

        # Covariance
        msg_est.pose.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.2
        ]

        self.pub_estimated.publish(msg_est)

        self.t += 0.05


def main():
    rclpy.init()
    node = PosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
