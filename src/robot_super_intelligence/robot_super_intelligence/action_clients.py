"""Typed operation adapters; the manager owns dispatch and cancellation."""

import math

from autonomy_msgs.action import Dig, Dump, ManageRobot
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


class ActionClients:
    def __init__(self, node):
        self.clients = {
            ManageRobot.Goal.NAVIGATE: ActionClient(node, NavigateToPose, 'navigate_to_pose'),
            ManageRobot.Goal.DIG: ActionClient(node, Dig, 'dig'),
            ManageRobot.Goal.DUMP: ActionClient(node, Dump, 'dump'),
        }
        self.sources = {
            ManageRobot.Goal.NAVIGATE: 'navigation',
            ManageRobot.Goal.DIG: 'dig',
            ManageRobot.Goal.DUMP: 'dump',
        }

    def valid(self, request):
        if not math.isfinite(request.time_limit_sec) or request.time_limit_sec <= 0:
            return False
        if request.operation == ManageRobot.Goal.FULL_AUTO:
            return True
        if request.operation not in self.clients:
            return False
        if request.operation in (ManageRobot.Goal.DIG, ManageRobot.Goal.DUMP):
            return request.routine_index == 0
        pose = request.target_pose
        p, q = pose.pose.position, pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        return (bool(pose.header.frame_id) and all(math.isfinite(v) for v in values)
                and abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) < 1e-3)

    def goal(self, request):
        if request.operation == ManageRobot.Goal.NAVIGATE:
            return NavigateToPose.Goal(pose=request.target_pose)
        action = Dig if request.operation == ManageRobot.Goal.DIG else Dump
        return action.Goal(index=request.routine_index)
