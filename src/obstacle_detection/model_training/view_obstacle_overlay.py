"""View the blue RGB camera with the rock mask overlaid at 50% opacity."""

import cv2
import numpy as np
import rclpy
from message_filters import Subscriber, TimeSynchronizer
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class ObstacleOverlayViewer(Node):
    """Display timestamp-matched camera and mask images without resizing."""

    def __init__(self):
        super().__init__("obstacle_overlay_viewer")
        self.camera_sub = Subscriber(
            self, Image, "/rgbd_blue/color/image_raw",
            qos_profile=qos_profile_sensor_data,
        )
        self.mask_sub = Subscriber(
            self, Image, "/obstacles/blue/mask",
            qos_profile=qos_profile_sensor_data,
        )
        self.sync = TimeSynchronizer([self.camera_sub, self.mask_sub], 10)
        self.sync.registerCallback(self.show_overlay)
        cv2.namedWindow("Blue camera + rock mask (50% opacity)", cv2.WINDOW_AUTOSIZE)
        self.get_logger().info(
            "Showing timestamp-matched blue camera frames and masks at native size"
        )

    @staticmethod
    def image_to_bgr(message):
        rows = np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height, message.step
        )

        if message.encoding == "mono8":
            gray = rows[:, :message.width]
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        if message.encoding in ("rgb8", "bgr8"):
            pixels = rows[:, :message.width * 3].reshape(
                message.height, message.width, 3
            )
            if message.encoding == "rgb8":
                return cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
            return pixels.copy()
        raise ValueError(f"Unsupported camera encoding: {message.encoding}")

    def show_overlay(self, camera_message, mask_message):
        if mask_message.encoding != "mono8":
            self.get_logger().warning(
                f"Expected a mono8 mask, received {mask_message.encoding}"
            )
            return

        camera = self.image_to_bgr(camera_message)
        mask_rows = np.frombuffer(mask_message.data, dtype=np.uint8).reshape(
            mask_message.height, mask_message.step
        )
        mask = mask_rows[:, :mask_message.width]

        if camera.shape[:2] != mask.shape[:2]:
            self.get_logger().warning(
                "Camera and mask dimensions differ: "
                f"camera={camera.shape[1]}x{camera.shape[0]}, "
                f"mask={mask.shape[1]}x{mask.shape[0]}. "
                "Showing each at native size without overlay or scaling.",
                throttle_duration_sec=2.0,
            )
            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            side_by_side = np.zeros(
                (max(camera.shape[0], mask_bgr.shape[0]),
                 camera.shape[1] + mask_bgr.shape[1], 3),
                dtype=np.uint8,
            )
            side_by_side[:camera.shape[0], :camera.shape[1]] = camera
            side_by_side[:mask_bgr.shape[0], camera.shape[1]:] = mask_bgr
            cv2.imshow("Blue camera + rock mask (50% opacity)", side_by_side)
            cv2.waitKey(1)
            return

        # Class ID 1 is rock. Blend only those pixels so the camera stays at
        # full brightness everywhere else and the mask overlay is 50% opaque.
        rocks = mask == 1
        overlay = camera.copy()
        rock_color = np.zeros_like(camera)
        rock_color[:, :] = (0, 128, 255)  # orange in OpenCV's BGR order
        overlay[rocks] = cv2.addWeighted(
            camera[rocks], 0.5, rock_color[rocks], 0.5, 0.0
        )
        cv2.imshow("Blue camera + rock mask (50% opacity)", overlay)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = ObstacleOverlayViewer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
