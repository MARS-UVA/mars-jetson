"""AprilTag gate calls and robot pose lookups for localization states."""

from rclpy.clock import Clock, ClockType
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from robot_perception_interfaces.srv import AprilTagGate
from tf2_ros import Buffer, TransformException, TransformListener


class AprilTags:
    def __init__(self, node, gate_service='/AprilTagGate',
                 map_frame='map', base_frame='base_link'):
        self.node = node
        self.map_frame = map_frame
        self.base_frame = base_frame
        self.client = node.create_client(AprilTagGate, gate_service)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.pending = None

    def ready(self):
        return self.client.service_is_ready()

    def call_gate(self, timeout_sec, callback):
        """Call the gate once; callback(True) only if it forwarded detections.

        A call that has not answered after timeout_sec reports False.
        """
        self.cancel()
        call = {'done': False, 'timer': None}

        def finish(ok):
            if call['done']:
                return
            call['done'] = True
            if call['timer'] is not None:
                self.node.destroy_timer(call['timer'])
            if self.pending is call:
                self.pending = None
            callback(ok)

        def answered(future):
            try:
                ok = bool(future.result().success)
            except Exception as error:  # noqa: BLE001 - any failed call is a miss
                self.node.get_logger().warn(f'AprilTagGate call failed: {error}')
                ok = False
            finish(ok)

        self.pending = call
        call['timer'] = self.node.create_timer(
            timeout_sec, lambda: finish(False), clock=self.clock)
        self.client.call_async(AprilTagGate.Request()).add_done_callback(answered)

    def cancel(self):
        """Forget any pending call; its answer will be ignored."""
        call, self.pending = self.pending, None
        if call is not None and not call['done']:
            call['done'] = True
            self.node.destroy_timer(call['timer'])

    def current_pose(self):
        """The robot's pose in the map frame, or None if it is not known."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time())
        except TransformException:
            return None
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose
