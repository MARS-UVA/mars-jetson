"""ROS action adapters supervised by the manager, independent of machine states."""


import math

from autonomy_msgs.action import Dig, Dump
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from robot_control_msgs.msg import RobotState


class NavigationOperation:
    source = 'navigation'
    mode = RobotState.NAVIGATION

    def __init__(self, node):
        self.client = ActionClient(node, NavigateToPose, 'navigate_to_pose') if node else None

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


class DigOperation:
    source = 'dig'
    mode = RobotState.DIG

    def __init__(self, node):
        self.client = ActionClient(node, Dig, 'dig') if node else None

    def valid(self, request):
        return request.routine_index == Dig.Goal.BRUNO_INDEX

    def send_goal(self, request, feedback, on_response):
        future = self.client.send_goal_async(
            Dig.Goal(index=request.routine_index), feedback_callback=feedback)
        future.add_done_callback(on_response)


class DumpOperation:
    source = 'dump'
    mode = RobotState.DUMP

    def __init__(self, node):
        self.client = ActionClient(node, Dump, 'dump') if node else None

    def valid(self, request):
        return request.routine_index == Dump.Goal.BRUNO_INDEX

    def send_goal(self, request, feedback, on_response):
        future = self.client.send_goal_async(
            Dump.Goal(index=request.routine_index), feedback_callback=feedback)
        future.add_done_callback(on_response)
