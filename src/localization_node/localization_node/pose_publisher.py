#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped  # NEW


class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')

        # Publisher 1: original circular motion (now used for ZED->PoseWithCov)
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

        # NEW: Subscribe to ZED pose
        self.sub_zed_pose = self.create_subscription(
            PoseStamped,
            '/zed/zed_node/pose',
            self.zed_pose_cb,
            10
        )

        # NEW: Covariance for the ZED pose (6x6 row-major)
        # (x, y, z, rot_x, rot_y, rot_z)
        self.zed_covariance = [
            0.05, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.05, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.10, 0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.20, 0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.20, 0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.10
        ]

        # Timer for 10 Hz
        self.timer = self.create_timer(0.1, self.publish_pose)
        self.t = 0.0

    # NEW: Convert /zed/zed_node/pose -> /pose_with_cov
    def zed_pose_cb(self, msg: PoseStamped):
        out = PoseWithCovarianceStamped()

        # Keep ZED header (stamp + frame_id) so TF/sync works
        out.header = msg.header

        # Copy pose
        out.pose.pose = msg.pose

        # Add covariance
        out.pose.covariance = self.zed_covariance
        out.header.frame_id = 'odom'

        self.pub_cov.publish(out)

    def publish_pose(self):
        now = self.get_clock().now().to_msg()

        # --------------------------
        # Publisher 1: /pose_with_cov
        # --------------------------
        # COMMENTED OUT: replaced by ZED pose callback (zed_pose_cb)
        """
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
        """

        # --------------------------
        # Publisher 2: /localization/estimated_pose
        # --------------------------
        msg_est = PoseWithCovarianceStamped()
        msg_est.header.stamp = now
        msg_est.header.frame_id = 'odom'

        # Linear motion along x
        msg_est.pose.pose.position.x = math.cos(self.t)
        msg_est.pose.pose.position.y = math.sin(self.t)
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
