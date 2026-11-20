"""LocalizeToAprilTags: feed AprilTag sightings to RTAB-Map, turning to find one.

At each heading the gate is called. It forwards detections to RTAB-Map and
answers success only if it saw a tag; otherwise the robot turns in place to
the next heading (a navigation goal at its starting position) and tries
again. By default it checks straight ahead, then 45 deg to one side, then
45 deg to the other. Only the front camera detects tags.
"""

import math

from autonomy_msgs.action import ManageRobot
from geometry_msgs.msg import PoseStamped

from .state import State
from ..outcomes import Outcome


def rotated(pose, angle):
    """The same position as pose, with its heading turned by angle (rad)."""
    q = pose.pose.orientation
    yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z)) + angle
    goal = PoseStamped()
    goal.header.frame_id = pose.header.frame_id
    goal.pose.position.x = pose.pose.position.x
    goal.pose.position.y = pose.pose.position.y
    goal.pose.position.z = pose.pose.position.z
    goal.pose.orientation.z = math.sin(yaw / 2)
    goal.pose.orientation.w = math.cos(yaw / 2)
    return goal


class LocalizeToAprilTags(State):
    outcomes = frozenset({Outcome.APRILTAG_LOCALIZED, Outcome.APRILTAG_DETECTION_FAILED})

    def __init__(self, apriltags, headings_rad=(math.radians(45), math.radians(-45)),
                 gate_timeout_sec=15.0):
        """Check straight ahead, then each heading in headings_rad.

        Headings are relative to the robot's heading when the search starts.
        gate_timeout_sec only guards against a gate that never answers; it
        must exceed a full gate batch (the gate itself gives up after about
        2 s without a tag).
        """
        if apriltags is None:
            raise ValueError('An AprilTag helper is required')
        headings_rad = tuple(headings_rad)
        if not (all(math.isfinite(h) for h in headings_rad)
                and math.isfinite(gate_timeout_sec) and gate_timeout_sec > 0):
            raise ValueError('Invalid localization settings')
        self.apriltags = apriltags
        self.headings_rad = headings_rad
        self.gate_timeout_sec = gate_timeout_sec
        self.enter()

    def enter(self):
        self.turns = 0
        self.origin = None
        self.active = False

    def start(self, dispatch, on_complete):
        self.dispatch = dispatch
        self.on_complete = on_complete
        self.active = True
        self.check()

    def cancel(self):
        self.active = False
        self.apriltags.cancel()

    def check(self):
        self.apriltags.call_gate(self.gate_timeout_sec, self.gate_answered)

    def gate_answered(self, saw_tag):
        if not self.active:
            return
        if saw_tag:
            self.finish(Outcome.APRILTAG_LOCALIZED)
        elif self.turns >= len(self.headings_rad):
            self.finish(Outcome.APRILTAG_DETECTION_FAILED)
        else:
            self.turn()

    def turn(self):
        # Remember where the search started; every heading is measured from it.
        if self.origin is None:
            self.origin = self.apriltags.current_pose()
            if self.origin is None:
                self.finish(Outcome.APRILTAG_DETECTION_FAILED)
                return
        target = rotated(self.origin, self.headings_rad[self.turns])
        self.dispatch(ManageRobot.Goal(operation=ManageRobot.Goal.NAVIGATE,
                                       target_pose=target), self.turned)

    def turned(self, outcome):
        if not self.active:
            return
        if outcome != 'SUCCESS':
            self.finish(Outcome.APRILTAG_DETECTION_FAILED)
            return
        self.turns += 1
        self.check()

    def finish(self, outcome):
        self.active = False
        self.on_complete(outcome)
