#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class SolidColorPublisher(Node):
    def __init__(self):
        super().__init__('solid_color_publisher')

        # Parameters
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('r', 0)
        self.declare_parameter('g', 255)
        self.declare_parameter('b', 0)
        self.declare_parameter('topic', 'awareness/image_raw')
        self.declare_parameter('publish_rate', 10.0)

        width = int(self.get_parameter('width').value)
        height = int(self.get_parameter('height').value)
        r = int(self.get_parameter('r').value)
        g = int(self.get_parameter('g').value)
        b = int(self.get_parameter('b').value)
        topic_name = self.get_parameter('topic').value
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.publisher = self.create_publisher(Image, topic_name, 10)

        # --- Create raw pixel buffer for solid RGB image (no numpy needed) ---
        pixel = bytes([r, g, b])                           # One pixel
        pixel_row = pixel * width                          # One row
        pixel_data = pixel_row * height                    # Full image

        # --- Create Image message ---
        self.image_msg = Image()
        self.image_msg.header.frame_id = "camera"
        self.image_msg.height = height
        self.image_msg.width = width
        self.image_msg.encoding = "rgb8"
        self.image_msg.is_bigendian = 0
        self.image_msg.step = width * 3
        self.image_msg.data = pixel_data

        # Timer
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_image)

        self.get_logger().info(
            f"Publishing solid RGB({r},{g},{b}) {width}x{height} image on '{topic_name}'"
        )

    def publish_image(self):
        # Update timestamp each publish
        self.image_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.image_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SolidColorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

