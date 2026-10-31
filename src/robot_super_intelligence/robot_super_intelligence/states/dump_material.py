"""DumpMaterial state."""

from autonomy_msgs.action import Dump
from rclpy.action import ActionClient
from robot_control_msgs.msg import RobotState

from .base import Placeholder
from ..outcomes import Outcome


class DumpMaterial(Placeholder):
    source = 'dump'
    mode = RobotState.DUMP
    outcomes = frozenset({Outcome.DUMP_SUCCESS, Outcome.FAILED, Outcome.NOT_IMPLEMENTED})

    def __init__(self, node=None):
        self.client = ActionClient(node, Dump, 'dump') if node else None

    def valid(self, request):
        return request.routine_index == Dump.Goal.BRUNO_INDEX

    def send_goal(self, request, feedback):
        return self.client.send_goal_async(
            Dump.Goal(index=request.routine_index), feedback_callback=feedback)
