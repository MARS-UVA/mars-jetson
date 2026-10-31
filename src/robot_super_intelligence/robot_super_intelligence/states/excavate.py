"""Excavate state."""

from autonomy_msgs.action import Dig
from rclpy.action import ActionClient
from robot_control_msgs.msg import RobotState

from .base import Placeholder
from ..outcomes import Outcome


class Excavate(Placeholder):
    source = 'dig'
    mode = RobotState.DIG
    outcomes = frozenset({Outcome.EXCAVATE_SUCCESS, Outcome.FAILED, Outcome.NOT_IMPLEMENTED})

    def __init__(self, node=None):
        self.client = ActionClient(node, Dig, 'dig') if node else None

    def valid(self, request):
        return request.routine_index == Dig.Goal.BRUNO_INDEX

    def send_goal(self, request, feedback, on_response):
        future = self.client.send_goal_async(
            Dig.Goal(index=request.routine_index), feedback_callback=feedback)
        future.add_done_callback(on_response)
