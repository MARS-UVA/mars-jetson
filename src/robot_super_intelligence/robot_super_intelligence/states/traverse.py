"""Nav2 client shared by traversal states and standalone navigation."""

import math

from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from robot_control_msgs.msg import RobotState

from .state import State
from ..outcomes import Outcome


class Traverse(State):
    source = 'navigation'
    mode = RobotState.NAVIGATION
    outcomes = frozenset({Outcome.FAILED, Outcome.NOT_IMPLEMENTED})

    def __init__(self, node=None):
        self.client = ActionClient(node, NavigateToPose, 'navigate_to_pose') if node else None

    def start(self, dispatch, on_complete):
        # Machine traversal states supply a destination and override this method.
        # Standalone navigation uses send_goal() with the request's target pose.
        on_complete(Outcome.NOT_IMPLEMENTED)

    def valid(self, request):
        pose = request.target_pose
        p, q = pose.pose.position, pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        return (bool(pose.header.frame_id) and all(math.isfinite(v) for v in values)
                and abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) < 1e-3)

    def send_goal(self, request, feedback, on_response):
        future = self.client.send_goal_async(
            NavigateToPose.Goal(pose=request.target_pose), feedback_callback=feedback)
        future.add_done_callback(on_response)
