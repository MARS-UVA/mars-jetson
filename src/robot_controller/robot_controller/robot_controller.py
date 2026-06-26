import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from serial_msgs.msg import MotorCommands
from sensor_msgs.msg import JointState

from geometry_msgs.msg import Twist

CONTROL_HZ = 20
NEUTRAL = 127

class RobotControllerNode(Node):
    """A ROS 2 node which controls the robot's movement based on Twist and JointState messages."""
    
    def __init__(self, **kwargs):
        super().__init__('robot_controller', **kwargs)

        self.motor_commands_buffer = {
            'front_left': NEUTRAL,
            'back_left': NEUTRAL,
            'front_right': NEUTRAL,
            'back_right': NEUTRAL,
            'front_drum': NEUTRAL,
            'front_arm': NEUTRAL,
            'back_drum': NEUTRAL,
            'back_arm': NEUTRAL
        }

        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.RELIABLE)
        )
        self.arm_drum_state_subscriber = self.create_subscription(
            JointState,
            '/arm_drum_state',
            self.arm_drum_state_callback,
            QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.RELIABLE)
        )

        self.motor_commands_publisher = self.create_publisher(
            MotorCommands,
            '/motor_commands',
            QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.RELIABLE)
        )
        self.timer = self.create_timer(1.0 / CONTROL_HZ, self.publish_motor_commands)
    
    def publish_motor_commands(self) -> None:
        motor_commands_msg = MotorCommands(motor_commands=list(self.motor_commands_buffer.values()))
        self.motor_commands_publisher.publish(motor_commands_msg)

    def convert_velocity_to_motor_command(self, velocity: float) -> int:
        """Convert a velocity value (-1.0 to 1.0) to a motor command (0-255)."""
        return int((velocity + 1) * 127.5)

    def cmd_vel_callback(self, msg: Twist) -> None:
        """Callback for Twist messages to control the robot's movement."""
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Calculate wheel speeds based on linear and angular velocities
        left_speed = linear_x - angular_z
        right_speed = linear_x + angular_z

        left_motor_command = self.convert_velocity_to_motor_command(left_speed)
        right_motor_command = self.convert_velocity_to_motor_command(right_speed)

        # Convert speeds to motor command values (0-255)
        self.motor_commands_buffer['front_left'] = left_motor_command
        self.motor_commands_buffer['back_left'] = left_motor_command
        self.motor_commands_buffer['front_right'] = right_motor_command
        self.motor_commands_buffer['back_right'] = right_motor_command

    def arm_drum_state_callback(self, msg: JointState) -> None:
        """Callback for JointState messages to control the robot's arm and drum."""
        for name, speed in zip(msg.name, msg.velocity):
            if name not in self.motor_commands_buffer:
                self.get_logger().warn(f"Received unknown joint name: {name}")
                continue

            self.motor_commands_buffer[name] = self.convert_velocity_to_motor_command(speed)

def main() -> None:
    rclpy.init(args=sys.argv)
    node = RobotControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
