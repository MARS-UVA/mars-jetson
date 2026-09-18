#!/usr/bin/env python3
"""Convert measured Gazebo arm angles to calibrated actuator feedback."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from serial_msgs.msg import Position


def calibrated_position(angle, angle_start, angle_end, output_start, output_end):
    fraction = min(1.0, max(0.0, (angle - angle_start) / (angle_end - angle_start)))
    return output_start + fraction * (output_end - output_start)


class ActuatorPositionFeedback(Node):
    def __init__(self):
        super().__init__('actuator_position_feedback')
        self.calibration = {}
        for side, joint in [('front', 'front_arm_joint'), ('back', 'rear_arm_joint')]:
            defaults = {
                'joint': joint,
                'angle_start': -1.2,
                'angle_end': 0.2,
                'output_start': 0.0,
                'output_end': 1.0,
            }
            values = {key: self.declare_parameter(f'{side}.{key}', value).value
                      for key, value in defaults.items()}
            if (not all(math.isfinite(values[key]) for key in defaults if key != 'joint')
                    or values['angle_start'] == values['angle_end']):
                raise ValueError(f'Invalid {side} actuator calibration')
            self.calibration[side] = values
        self.publisher = self.create_publisher(Position, 'position', 10)
        self.subscription = self.create_subscription(
            JointState, 'joint_states', self.on_joint_states, qos_profile_sensor_data)

    def on_joint_states(self, message):
        positions = dict(zip(message.name, message.position))
        output = Position()
        for side, config in self.calibration.items():
            angle = positions.get(config['joint'])
            if angle is None or not math.isfinite(angle):
                return  # Never publish fabricated or partially stale positions.
            value = calibrated_position(
                angle, config['angle_start'], config['angle_end'],
                config['output_start'], config['output_end'])
            setattr(output, f'{side}_actuator_position', value)
        self.publisher.publish(output)


def main():
    rclpy.init()
    node = ActuatorPositionFeedback()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
