"""Opens the AprilTag gate for a few seconds at the control station's request.

robot_perception's AprilTagGate only forwards /apriltag/detections to
/detections (which RTAB-Map listens to) while its /AprilTagGate service is
being called; each call forwards about send_detections messages. This node
keeps calling it back to back until the requested time is up, so RTAB-Map
gets a burst of tag sightings to localize from.

The control station can't call /AprilTagGate itself: rosbridge would need
robot_perception_interfaces built on its side. So this node uses std_msgs:

Subscribes:  /apriltag_gate/open    std_msgs/Float32  seconds to keep it open
                                    (clamped to 1..max_duration; pressing
                                    again while open extends it)
Publishes:   /apriltag_gate/status  std_msgs/String   JSON, 2 Hz and on change
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from robot_perception_interfaces.srv import AprilTagGate
from std_msgs.msg import Float32, String


class AprilTagGateTrigger(Node):

    def __init__(self):
        super().__init__('apriltag_gate_trigger')
        self.max_duration = float(self.declare_parameter('max_duration', 30.0).value)
        gate_service = self.declare_parameter('gate_service', '/AprilTagGate').value

        self.client = self.create_client(AprilTagGate, gate_service)
        self.status_pub = self.create_publisher(String, '/apriltag_gate/status', 10)
        self.create_subscription(Float32, '/apriltag_gate/open', self.on_open, 10)
        self.create_timer(0.5, self.tick)

        self.open_until = 0.0     # time.monotonic() deadline; wall clock, not sim time
        self.call_pending = False
        self.calls_done = 0
        self.last_message = 'closed'

    def on_open(self, msg):
        seconds = msg.data
        if not math.isfinite(seconds):
            return
        seconds = min(max(seconds, 1.0), self.max_duration)
        if not self.client.service_is_ready():
            self.last_message = 'gate service not available'
            self.publish_status()
            return
        self.open_until = max(self.open_until, time.monotonic() + seconds)
        self.last_message = 'open'
        self.get_logger().info(f'Opening AprilTag gate for {seconds:.0f} s')
        self.call_gate()
        self.publish_status()

    def call_gate(self):
        # The gate's callback blocks until it has forwarded its batch, so only
        # one call is ever outstanding; the next starts when it returns.
        if self.call_pending or time.monotonic() >= self.open_until:
            return
        self.call_pending = True
        self.client.call_async(AprilTagGate.Request()).add_done_callback(self.on_gate_done)

    def on_gate_done(self, future):
        self.call_pending = False
        try:
            ok = future.result().success
        except Exception as error:  # noqa: BLE001 - report any failed call
            ok = False
            self.get_logger().warn(f'AprilTagGate call failed: {error}')
        if not ok:
            # Don't hammer a failing service; close and let the operator retry.
            self.open_until = 0.0
            self.last_message = 'gate call failed'
            self.publish_status()
            return
        self.calls_done += 1
        if time.monotonic() < self.open_until:
            self.call_gate()
        else:
            self.last_message = 'closed'
        self.publish_status()

    def tick(self):
        if not self.call_pending and time.monotonic() >= self.open_until \
                and self.last_message == 'open':
            self.last_message = 'closed'
        self.publish_status()

    def publish_status(self):
        remaining = max(0.0, self.open_until - time.monotonic())
        if remaining > 0:
            state = 'open'
        elif self.call_pending:
            # Time is up but the gate is still waiting for tag sightings to
            # finish its batch (e.g. the tag is out of view). It can't be
            # cancelled; it closes once the batch is forwarded.
            state = 'finishing'
        else:
            state = 'closed'
        status = {
            'state': state,
            'remaining': round(remaining, 1),
            'batches': self.calls_done,
            'gate_available': self.client.service_is_ready(),
            'message': self.last_message,
        }
        self.status_pub.publish(String(data=json.dumps(status)))


def main():
    rclpy.init()
    node = AprilTagGateTrigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
