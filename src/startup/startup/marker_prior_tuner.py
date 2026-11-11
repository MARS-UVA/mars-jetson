"""Lets the control station shift the AprilTag priors RTAB-Map anchors to.

Subscribes:  /marker_priors/set_offset  geometry_msgs/Point  absolute offset (m)
                                         added to every tag's x, y, z.
Publishes:   /marker_priors/state       std_msgs/String  JSON, 1 Hz and on change.

On each new offset it sets Marker/Priors on the RTAB-Map node and calls its
update_parameters service. update_parameters re-reads EVERY RTAB-Map setting
from the node's ROS parameters, and settings that came from the .ini file or
rtabmap_args are not ROS parameters, so they would silently fall back to
defaults (e.g. Optimizer/PriorsIgnored -> true, which disables priors). To
avoid that, gazebo.launch.py hands this node those settings as
rtabmap_settings, and they are re-sent alongside Marker/Priors.
"""
import json
import math

import rclpy
from geometry_msgs.msg import Point
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty

POSE_FIELDS = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')


def parse_priors(text):
    """'id x y z r p y|...' -> [(id, [x, y, z, r, p, y]), ...]"""
    tags = []
    for entry in filter(None, (e.strip() for e in text.split('|'))):
        fields = entry.split()
        tags.append((int(fields[0]), [float(v) for v in fields[1:7]]))
    return tags


def format_priors(tags):
    return '|'.join(f'{tag_id} ' + ' '.join(repr(v) for v in pose) for tag_id, pose in tags)


class MarkerPriorTuner(Node):

    def __init__(self):
        super().__init__('marker_prior_tuner')
        self.base = parse_priors(self.declare_parameter('base_priors', '').value)
        self.settings = json.loads(self.declare_parameter('rtabmap_settings', '{}').value)
        self.max_offset = float(self.declare_parameter('max_offset', 0.5).value)
        rtabmap_node = self.declare_parameter('rtabmap_node', '/rtabmap/rtabmap').value
        update_service = self.declare_parameter(
            'update_service', '/rtabmap/update_parameters').value

        self.offset = [0.0, 0.0, 0.0]
        self.status = 'base priors (no offset)'
        self.busy = False
        self.dirty = False

        self.set_client = self.create_client(SetParameters, f'{rtabmap_node}/set_parameters')
        self.update_client = self.create_client(Empty, update_service)
        self.state_pub = self.create_publisher(String, '/marker_priors/state', 10)
        self.create_subscription(Point, '/marker_priors/set_offset', self.on_offset, 10)
        self.create_timer(1.0, self.publish_state)

    def shifted(self):
        return [(i, [p[0] + self.offset[0], p[1] + self.offset[1], p[2] + self.offset[2], *p[3:]])
                for i, p in self.base]

    def on_offset(self, msg):
        offset = [msg.x, msg.y, msg.z]
        if not self.base:
            self.status = 'rejected: no base priors configured'
        elif not all(math.isfinite(v) and abs(v) <= self.max_offset for v in offset):
            self.status = f'rejected: offset must be finite and within ±{self.max_offset} m'
        else:
            self.offset = offset
            self.apply()
        self.publish_state()

    def apply(self):
        if self.busy:
            self.dirty = True  # send the newest offset once the current call finishes
            return
        if not (self.set_client.service_is_ready() and self.update_client.service_is_ready()):
            self.status = 'not applied: RTAB-Map is not running'
            return
        values = dict(self.settings)
        values['Marker/Priors'] = format_priors(self.shifted())
        request = SetParameters.Request(parameters=[
            Parameter(name=name, value=ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value=str(value)))
            for name, value in values.items()
        ])
        self.busy = True
        self.status = 'applying…'
        self.set_client.call_async(request).add_done_callback(self.on_set_done)

    def on_set_done(self, future):
        failed = [r.reason for r in future.result().results if not r.successful]
        if failed:
            self.finish(f'not applied: RTAB-Map rejected parameters ({failed[0]})')
            return
        self.update_client.call_async(Empty.Request()).add_done_callback(
            lambda _: self.finish('applied'))

    def finish(self, status):
        self.busy = False
        self.status = status
        self.get_logger().info(f'Marker priors {status}: offset {self.offset}')
        if self.dirty:
            self.dirty = False
            self.apply()
        self.publish_state()

    def publish_state(self):
        state = {
            'offset': dict(zip('xyz', self.offset)),
            'max_offset': self.max_offset,
            'status': self.status,
            'tags': [{'id': i, **dict(zip(POSE_FIELDS, p))} for i, p in self.shifted()],
        }
        self.state_pub.publish(String(data=json.dumps(state)))


def main():
    rclpy.init()
    node = MarkerPriorTuner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
