"""Excavate state."""

from autonomy_msgs.action import Dig, ManageRobot
from rclpy.action import ActionClient
from robot_control_msgs.msg import RobotState

from .base import Placeholder
from ..outcomes import Outcome


class Excavate(Placeholder):
    source = 'dig'
    mode = RobotState.DIG
    outcomes = frozenset({Outcome.EXCAVATE_SUCCESS, Outcome.FAILED, Outcome.NOT_IMPLEMENTED})

    def __init__(self, node=None, routine_index=None):
        self.client = ActionClient(node, Dig, 'dig') if node else None
        self.routine_index = routine_index

    def start(self, dispatch, on_complete):
        if self.routine_index is None:
            return super().start(dispatch, on_complete)
        dispatch(ManageRobot.Goal(operation=ManageRobot.Goal.DIG,
                                  routine_index=self.routine_index),
                 lambda outcome: on_complete(Outcome.EXCAVATE_SUCCESS
                     if outcome == 'SUCCESS' else Outcome.FAILED))

    def valid(self, request):
        return request.routine_index == Dig.Goal.BRUNO_INDEX

    def send_goal(self, request, feedback, on_response):
        future = self.client.send_goal_async(
            Dig.Goal(index=request.routine_index), feedback_callback=feedback)
        future.add_done_callback(on_response)
