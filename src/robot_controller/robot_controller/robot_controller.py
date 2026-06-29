import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from serial_msgs.msg import MotorCommands
from sensor_msgs.msg import JointState

from geometry_msgs.msg import Twist

CONTROL_HZ = 20
NEUTRAL = 127
MAXIMUM_DRUM_SPEED = 2.0
MAXIMUM_ARM_SPEED = 2.0
MAXIMUM_DRIVE_SPEED = 10.0

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
    
    def convert_speed_to_motor_command(self, speed: float) -> int:
        """Convert a speed value (-1.0 to 1.0) to a motor command (0-255)."""
        cmd = int((speed + 1) * 127.5)
        if cmd < 0 or cmd > 255:
            self.get_logger().warn(f"Motor command {cmd} out of bounds, clamping to [0, 255]")
            cmd = max(0, min(255, cmd))

        return cmd

    def cmd_vel_callback(self, msg: Twist) -> None:
        """Callback for Twist messages to control the robot's movement."""
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Calculate wheel speeds based on linear and angular velocities
        left_speed = linear_x - angular_z
        right_speed = linear_x + angular_z

        left_motor_command = self.convert_speed_to_motor_command(left_speed / MAXIMUM_DRIVE_SPEED)
        right_motor_command = self.convert_speed_to_motor_command(right_speed / MAXIMUM_DRIVE_SPEED)

        # Convert speeds to motor command values (0-255)
        self.motor_commands_buffer['front_left'] = left_motor_command
        self.motor_commands_buffer['back_left'] = left_motor_command
        self.motor_commands_buffer['front_right'] = right_motor_command
        self.motor_commands_buffer['back_right'] = right_motor_command

    def arm_drum_control_callback(self, msg: ArmDrumControl) -> None:
        """Callback for ArmDrumControl messages to control the robot's arm and drum."""
        self.motor_commands_buffer['front_drum'] = self.convert_speed_to_motor_command(msg.front_drum_speed / MAXIMUM_DRUM_SPEED)
        self.motor_commands_buffer['front_actuator'] = self.convert_speed_to_motor_command(msg.front_arm_speed / MAXIMUM_ARM_SPEED)
        self.motor_commands_buffer['back_drum'] = self.convert_speed_to_motor_command(msg.back_drum_speed / MAXIMUM_DRUM_SPEED)
        self.motor_commands_buffer['back_actuator'] = self.convert_speed_to_motor_command(msg.back_arm_speed / MAXIMUM_ARM_SPEED)

def main() -> None:
    rclpy.init(args=sys.argv)
    node = RobotControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
