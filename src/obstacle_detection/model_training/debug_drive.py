#!/usr/bin/env python3
"""Interactive timed WASD driving through the robot's teleop velocity input."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


LINEAR_SPEED = 0.3  # m/s; used for w and s
TURN_SPEED = 0.3  # rad/s; used for a and d
PUBLISH_RATE_HZ = 20.0

COMMANDS = {
    'w': (1.0, 0.0),
    's': (-1.0, 0.0),
    'a': (0.0, 1.0),
    'd': (0.0, -1.0),
}


def publish_for(node, publisher, direction, duration):
    linear_scale, angular_scale = COMMANDS[direction]
    command = Twist()
    command.linear.x = linear_scale * LINEAR_SPEED
    command.angular.z = angular_scale * TURN_SPEED

    stop = Twist()
    deadline = time.monotonic() + duration
    while rclpy.ok() and time.monotonic() < deadline:
        publisher.publish(command)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(1.0 / PUBLISH_RATE_HZ)

    # Send stop several times so the mux/controller receives the final command.
    for _ in range(3):
        publisher.publish(stop)
        time.sleep(0.05)


def main():
    rclpy.init()
    node = Node('debug_drive')
    publisher = node.create_publisher(Twist, '/cmd_vel/teleop', 10)

    print('Timed drive: w=forward, s=backward, a=turn left, d=turn right; q=quit')
    print('Teleop must be the selected velocity source.')

    try:
        # Let ROS discovery connect this publisher to the mux input.
        time.sleep(0.5)
        while rclpy.ok():
            direction = input('Direction [w/a/s/d, q]: ').strip().lower()
            if direction == 'q':
                break
            if direction not in COMMANDS:
                print('Enter w, a, s, d, or q.')
                continue

            try:
                duration = float(input('Duration in seconds: ').strip())
            except ValueError:
                print('Enter a number greater than zero.')
                continue
            if not math.isfinite(duration) or duration <= 0.0:
                print('Enter a finite duration greater than zero.')
                continue

            publish_for(node, publisher, direction, duration)
    except (EOFError, KeyboardInterrupt):
        print('\nStopping.')
    finally:
        for _ in range(3):
            publisher.publish(Twist())
            time.sleep(0.05)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()