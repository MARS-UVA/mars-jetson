"""Mux drive and arm/drum commands under one ownership grant and stop lock."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from robot_control_msgs.srv import SetControl
from std_msgs.msg import Bool, Float64MultiArray

from .arbitration import Arbitration


class CommandMux(Node):
    def __init__(self):
        super().__init__('command_mux')
        sources = self.declare_parameter('sources', ['teleop', 'navigation', 'dig', 'dump']).value
        arm_sources = self.declare_parameter('arm_drum_sources', ['teleop', 'dig', 'dump']).value
        priorities = {source: self.declare_parameter(
            f'priorities.{source}', 100 if source == 'teleop' else 50).value
            for source in sources}
        timeout = self.declare_parameter('command_timeout_sec', 0.5).value
        self.heartbeat_timeout = self.declare_parameter('manager_timeout_sec', 0.5).value
        self.arbiter = Arbitration(priorities, timeout)
        self.heartbeat = None
        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.arm_pub = self.create_publisher(
            Float64MultiArray, 'arm_drum_controller/commands', 1)
        self.inputs = []
        for source in sources:
            self.inputs.append(self.create_subscription(
                Twist, f'cmd_vel/{source}',
                lambda msg, src=source: self.receive_drive(src, msg), 1))
            if source in arm_sources:
                self.inputs.append(self.create_subscription(
                    Float64MultiArray, f'arm_drum_control/{source}',
                    lambda msg, src=source: self.receive_arm(src, msg), 1))
        lock_topic = self.declare_parameter('lock.topic', 'control/locks/stop').value
        self.arbiter.stop_lock = self.declare_parameter('lock.initially_locked', True).value
        self.lock_sub = self.create_subscription(
            Bool, lock_topic, self.on_lock,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.heartbeat_sub = self.create_subscription(
            Bool, 'control/heartbeat', self.on_heartbeat, 1)
        self.service = self.create_service(SetControl, 'control/select', self.select)
        self.timer = self.create_timer(
            0.05, self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive_drive(self, source, message):
        values = (message.linear.x, message.linear.y, message.linear.z,
                  message.angular.x, message.angular.y, message.angular.z)
        if all(math.isfinite(v) for v in values):
            self.arbiter.receive('drive', source, message)
        else:
            self.arbiter.commands.pop(('drive', source), None)

    def receive_arm(self, source, message):
        if len(message.data) == 4 and all(math.isfinite(v) for v in message.data):
            self.arbiter.receive('arm_drum', source, message)
        else:
            self.arbiter.commands.pop(('arm_drum', source), None)

    def on_heartbeat(self, message):
        if message.data:
            self.heartbeat = time.monotonic()

    def on_lock(self, message):
        self.arbiter.stop_lock = message.data
        self.arbiter.commands.clear()
        self.publish()

    def healthy(self):
        return (self.heartbeat is not None
                and time.monotonic() - self.heartbeat < self.heartbeat_timeout)

    def select(self, request, response):
        if not request.locked and not self.healthy():
            response.message = 'Manager heartbeat unavailable'
            return response
        try:
            self.arbiter.select(request.sources, request.locked)
            self.publish()
            response.success = True
        except ValueError as error:
            response.message = str(error)
        return response

    def publish(self):
        if not self.healthy():
            # A recovered heartbeat must not restore a previous grant.
            self.arbiter.select([], True)
        commands = self.arbiter.outputs()
        self.velocity_pub.publish(commands.get('drive') or Twist())
        self.arm_pub.publish(commands.get('arm_drum') or
                             Float64MultiArray(data=[0.0] * 4))


def main(args=None):
    rclpy.init(args=args)
    node = CommandMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.arbiter.select([], True)
            node.publish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
