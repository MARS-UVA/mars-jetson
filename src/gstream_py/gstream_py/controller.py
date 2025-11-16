#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gstream_msgs.msg import CameraState
import sys
import termios
import tty
import select


class CameraKeyboardController(Node):
    def __init__(self):
        super().__init__('camera_keyboard_controller')
        self.publisher_ = self.create_publisher(CameraState, 'camera/stream_control', 10)
        self.get_logger().info("Camera keyboard controller started. Press 'w' to activate camera1, 'u' to deactivate, 'q' to quit.")

        # Configure keyboard input (non-blocking)
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        msg = CameraState()

        try:
            while rclpy.ok():
                key = self.get_key()
                if key == 'w':
                    msg.camera1 = msg.ACTIVE
                    self.publisher_.publish(msg)
                    self.get_logger().info("Camera 1 activated.")
                elif key == 'u':
                    msg.camera1 = msg.INACTIVE
                    self.publisher_.publish(msg)
                    self.get_logger().info("Camera 1 deactivated.")
                elif key == 'q':
                    self.get_logger().info("Exiting camera controller.")
                    break
        except KeyboardInterrupt:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = CameraKeyboardController()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
