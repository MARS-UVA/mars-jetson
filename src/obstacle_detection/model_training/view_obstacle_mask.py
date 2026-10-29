"""Display the obstacle class mask with class ID 1 scaled to white."""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class ObstacleMaskViewer(Node):
    """Show the blue camera's obstacle mask without publishing a ROS topic."""

    def __init__(self):
        super().__init__("obstacle_mask_viewer")
        self.create_subscription(
            Image,
            "/obstacles/blue/mask",
            self.show_mask,
            qos_profile_sensor_data,
        )

    def show_mask(self, message):
        if message.encoding != "mono8":
            self.get_logger().warning(
                f"Expected mono8 mask, received {message.encoding}"
            )
            return

        rows = np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height, message.step
        )
        class_ids = rows[:, :message.width]
        display = np.where(class_ids == 1, 255, 0).astype(np.uint8)
        cv2.imshow("Obstacle mask (white = rock)", display)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = ObstacleMaskViewer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
