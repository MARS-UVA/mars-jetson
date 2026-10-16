#!/usr/bin/env python3
"""Remove body/floor returns without waiting for LiDAR-derived odometry."""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


def transform_points(points, transform):
    q = transform.rotation
    quaternion = np.array([q.x, q.y, q.z, q.w], dtype=float)
    quaternion /= np.linalg.norm(quaternion)
    x, y, z, w = quaternion
    rotation = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])
    t = transform.translation
    return points @ rotation.T + np.array([t.x, t.y, t.z])


def obstacle_mask(points, half_length, half_width, body_height, min_height, max_height):
    finite = np.isfinite(points).all(axis=1)
    body = ((np.abs(points[:, 0]) <= half_length) &
            (np.abs(points[:, 1]) <= half_width) & (points[:, 2] <= body_height))
    height = (points[:, 2] >= min_height) & (points[:, 2] <= max_height)
    return finite & ~body & height


class NavigationCloudFilter(Node):
    def __init__(self):
        super().__init__('navigation_cloud_filter')
        defaults = {
            'input_topic': '/unilidar/cloud',
            'output_topic': '/unilidar/navigation_points',
            'base_frame': 'base_link',
            'body_half_length': 0.90, 'body_half_width': 0.38,
            'body_max_height': 0.50,
            'min_height_in_base': -0.08, 'max_height_in_base': 1.45,
        }
        self.settings = {key: self.declare_parameter(key, value).value
                         for key, value in defaults.items()}
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(
            PointCloud2, self.settings['output_topic'], qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            PointCloud2, self.settings['input_topic'], self.on_cloud, qos_profile_sensor_data)
        self.pending = None
        self.retry_timer = self.create_timer(0.02, self.process_pending)
        self.published = False

    def on_cloud(self, message):
        # Only retain the newest scan while waiting for the mounting transform.
        self.pending = message
        self.process_pending()

    def process_pending(self):
        message = self.pending
        if message is None:
            return
        settings = self.settings
        try:
            to_base = self.buffer.lookup_transform(
                settings['base_frame'], message.header.frame_id,
                Time.from_msg(message.header.stamp))
        except TransformException as error:
            self.get_logger().warning(
                f'Waiting for LiDAR mounting TF: {error}', throttle_duration_sec=5.0)
            return
        points = point_cloud2.read_points_numpy(
            message, field_names=('x', 'y', 'z'), skip_nans=False).reshape(-1, 3)
        base_points = transform_points(points, to_base.transform)
        mask = obstacle_mask(
            base_points, settings['body_half_length'], settings['body_half_width'],
            settings['body_max_height'], settings['min_height_in_base'],
            settings['max_height_in_base'])
        # Publish original coordinates/frame/stamp. Nav2's existing TF message
        # filter waits for delayed odom TF, and ray clearing uses the sensor origin.
        self.publisher.publish(point_cloud2.create_cloud_xyz32(
            message.header, points[mask].astype(np.float32)))
        self.pending = None
        if not self.published:
            self.get_logger().info(
                f'Publishing filtered LiDAR in {message.header.frame_id}; '
                'odom TF is handled by the costmap')
            self.published = True


def main(args=None):
    rclpy.init(args=args)
    node = NavigationCloudFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
