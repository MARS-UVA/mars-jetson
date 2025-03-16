import rclpy
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class RobotPoseListener(Node):
    def __init__(self, **kwargs):
        super().__init__('robot_pose_listener', **kwargs)

        self.__tf_buffer = Buffer()
        self.__tf_listener = TransformListener(self.__tf_buffer, self)

        self.__timer = self.create_timer(1 / 24, callback=self.__on_tick)

    def __on_tick(self) -> None:
        transform = self.__tf_buffer.lookup_transform('base_link', 'odom', time=Time())

        self.get_logger().info(f'{transform=}')


def main() -> None:
    rclpy.init()
    node = RobotPoseListener()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
