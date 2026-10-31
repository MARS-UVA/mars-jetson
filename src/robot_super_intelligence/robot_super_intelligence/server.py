"""Timed full-auto action with serialized, nonblocking callbacks."""

import math

import rclpy
from autonomy_msgs.action import FullAuto
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from robot_control_msgs.msg import RobotState
from std_msgs.msg import Float64MultiArray, UInt8

from .machine import Machine
from .outcomes import Outcome


class FullAutoServer(Node):
    """Accept explicit goals only while FULL_AUTO is selected."""

    def __init__(self):
        super().__init__('robot_super_intelligence')
        self.mode = None
        self.busy = False
        self.goal = None
        self.machine = None
        self.results = {}
        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel/full_auto', 1)
        self.arm_pub = self.create_publisher(
            Float64MultiArray, 'arm_drum_control/full_auto', 1)
        self.mode_pub = self.create_publisher(UInt8, 'robot_state/set', 1)
        self.mode_sub = self.create_subscription(
            RobotState, 'robot_state', self.on_mode,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.action_server = ActionServer(
            self, FullAuto, 'full_auto', self.execute,
            goal_callback=self.on_goal, cancel_callback=self.on_cancel,
            handle_accepted_callback=self.on_accepted)
        # Deadlines and cancellation continue while simulation time is paused.
        self.timer = self.create_timer(
            0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_goal(self, request):
        if (self.busy or self.mode != RobotState.FULL_AUTO
                or not math.isfinite(request.time_limit_sec)
                or request.time_limit_sec <= 0):
            return GoalResponse.REJECT
        self.busy = True
        return GoalResponse.ACCEPT

    def on_accepted(self, goal):
        self.goal = goal
        self.results[bytes(goal.goal_id.uuid)] = Future()
        goal.execute()
        try:
            self.machine = Machine(goal.request.time_limit_sec)
            if self.mode != RobotState.FULL_AUTO:
                self.machine.end(Outcome.MODE_EXITED)
            self.stop()
            self.feedback()
        except Exception as error:
            self.get_logger().error(f'Cannot start run: {error}')
            self.finish(Outcome.FAILED)

    async def execute(self, goal):
        key = bytes(goal.goal_id.uuid)
        try:
            return await self.results[key]
        finally:
            del self.results[key]

    def on_cancel(self, goal):
        return CancelResponse.ACCEPT

    def on_mode(self, message):
        self.mode = message.state
        if self.goal is not None and self.mode != RobotState.FULL_AUTO:
            try:
                self.machine.end(Outcome.MODE_EXITED)
            except Exception as error:
                self.get_logger().error(f'State cleanup failed: {error}')
            self.stop()

    def stop(self):
        self.velocity_pub.publish(Twist())
        self.arm_pub.publish(Float64MultiArray(data=[0.0] * 4))

    def feedback(self):
        self.goal.publish_feedback(FullAuto.Feedback(
            state=self.machine.current.__name__,
            latest_outcome=self.machine.latest_outcome,
            completed_cycles=self.machine.completed_cycles,
            elapsed_sec=self.machine.elapsed))

    def tick(self):
        # Keep the full-auto channels neutral even before a goal arrives.
        self.stop()
        if self.goal is None:
            return
        try:
            if self.goal.is_cancel_requested:
                self.machine.end(Outcome.CANCELED)
            self.machine.tick()
            self.feedback()
            if self.machine.done:
                self.finish(self.machine.result)
        except Exception as error:
            self.get_logger().error(f'Run failed: {error}')
            try:
                self.machine.end(Outcome.FAILED)
                self.feedback()
            finally:
                self.finish(Outcome.FAILED)

    def finish(self, outcome):
        goal = self.goal
        if goal is None:
            return
        self.stop()
        # This must be idempotent: the legacy ESTOP toggle can re-enable teleop.
        self.mode_pub.publish(UInt8(data=RobotState.ESTOP))
        result = FullAuto.Result(
            success=outcome is Outcome.TIME_LIMIT_REACHED,
            outcome=outcome.value,
            completed_cycles=self.machine.completed_cycles if self.machine else 0)
        if goal.is_cancel_requested:
            result.success = False
            result.outcome = Outcome.CANCELED.value
            goal.canceled()
        elif result.success:
            goal.succeed()
        else:
            # A mode change aborts the action: only an actual action cancel
            # request permits the ROS CANCELING -> CANCELED transition.
            goal.abort()
        self.results[bytes(goal.goal_id.uuid)].set_result(result)
        self.goal = None
        self.machine = None
        self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = FullAutoServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok() and node.goal is not None:
            try:
                node.machine.end(Outcome.SHUTDOWN)
            finally:
                node.finish(Outcome.SHUTDOWN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
