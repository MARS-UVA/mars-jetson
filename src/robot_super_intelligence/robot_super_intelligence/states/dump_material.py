"""DumpMaterial state."""

from autonomy_msgs.action import Dump, ManageRobot
from rclpy.action import ActionClient
from robot_control_msgs.msg import RobotState

from .base import Placeholder
from ..outcomes import Outcome


class DumpMaterial(Placeholder):
    source = 'dump'
    mode = RobotState.DUMP
    outcomes = frozenset({Outcome.DUMP_SUCCESS, Outcome.FAILED, Outcome.NOT_IMPLEMENTED})

    def __init__(self, node=None, routine_index=None):
        self.client = ActionClient(node, Dump, 'dump') if node else None
        self.routine_index = routine_index

    def start(self, dispatch, on_complete):
        if self.routine_index is None:
            return super().start(dispatch, on_complete)
        dispatch(ManageRobot.Goal(operation=ManageRobot.Goal.DUMP,
                                  routine_index=self.routine_index),
                 lambda outcome: on_complete(Outcome.DUMP_SUCCESS
                     if outcome == 'SUCCESS' else Outcome.FAILED))

    def valid(self, request):
        return request.routine_index == Dump.Goal.BRUNO_INDEX

    def send_goal(self, request, feedback, on_response):
        future = self.client.send_goal_async(
            Dump.Goal(index=request.routine_index), feedback_callback=feedback)
        future.add_done_callback(on_response)
