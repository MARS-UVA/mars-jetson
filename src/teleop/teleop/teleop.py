import copy
import sys

import rclpy
from rclpy.node import Node
from teleop_msgs.msg import GamepadState, MotorChanges

from .control import DriveControlStrategy, ArcadeDrive

from .motor_queries import wheel_speed_to_motor_queries


class TeleopNode(Node):

    def __init__(self, drive_control_strategy: DriveControlStrategy, **kwargs):
        super().__init__('teleop', **kwargs)
        self.__drive_control_strategy = copy.copy(drive_control_strategy)

        self._wheel_speed_publisher = self.create_publisher(
            msg_type=MotorChanges,
            topic = "tele-op",
            qos_profile = 10,
        )

        self.__gamepad_state_subscription = self.create_subscription(
            msg_type=GamepadState,
            topic='gamepad_state',
            callback=self.__on_receive_gamepad_state,
            qos_profile=10,
        )

    @property
    def drive_control_strategy(self) -> DriveControlStrategy:
        return self.__drive_control_strategy

    @drive_control_strategy.setter
    def drive_control_strategy(self, value: DriveControlStrategy) -> None:
        if not isinstance(value, DriveControlStrategy):
            raise ValueError('drive control strategy must be of type DriveControlStrategy')

        self.__drive_control_strategy = copy.copy(value)

    def __on_receive_gamepad_state(self, gamepad_state: GamepadState) -> None:
        wheel_speeds = self.__drive_control_strategy.get_wheel_speeds(gamepad_state)

        self.get_logger().info(f'Calculated: {wheel_speeds}')

        wheel_speed_msg = wheel_speed_to_motor_queries(wheel_speeds)
        self._wheel_speed_publisher.publish(wheel_speed_msg)


def main() -> None:
    rclpy.init(args=sys.argv)
    node = TeleopNode(ArcadeDrive())
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
