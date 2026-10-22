"""Action lifecycle and command publishing shared by the two routines."""

from dataclasses import dataclass

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from robot_control_msgs.msg import RobotState
from std_msgs.msg import Float64MultiArray, UInt8


@dataclass
class Command:
    """Arm/drum speeds in controller order, plus forward wheel velocity."""

    front_arm: float = 0.0
    back_arm: float = 0.0
    front_drum: float = 0.0
    back_drum: float = 0.0
    drive: float = 0.0


class RoutineServer(Node):
    """Advance a routine at 10 Hz without blocking subscription callbacks."""

    def __init__(self, name, action_type, mode, defaults, variants):
        super().__init__(f'{name}_node')
        self.action_type = action_type
        self.mode = mode
        self.variants = variants
        self.parameter_names = tuple(defaults)
        for key, value in defaults.items():
            self.declare_parameter(key, value)

        self.busy = False
        self.goal = None
        self.result_future = None
        self.results = {}
        self.routine = None
        self.phase = None
        self.state = None
        self.owns_mode = False
        self.velocity_pub = self.create_publisher(Twist, f'cmd_vel/{name}', 1)
        self.arm_pub = self.create_publisher(
            Float64MultiArray, f'arm_drum_control/{name}', 1)
        self.state_pub = self.create_publisher(UInt8, 'robot_state/toggle', 1)
        self.state_sub = self.create_subscription(
            RobotState, 'robot_state', self.on_state,
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.cancel_sub = self.create_subscription(
            UInt8, 'cancel_command', self.on_manual_cancel, 10)
        # Use the standard cancel service so manual cancellation follows the
        # action protocol's CANCELING -> CANCELED transition too.
        self.cancel_client = self.create_client(CancelGoal, f'{name}/_action/cancel_goal')
        self.action_server = ActionServer(
            self, action_type, name, self.execute,
            goal_callback=self.on_goal, cancel_callback=self.on_cancel,
            handle_accepted_callback=self.on_accepted)
        self.timer = self.create_timer(0.1, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_goal(self, request):
        if self.busy or request.index not in self.variants:
            return GoalResponse.REJECT
        # Reserve immediately, rather than waiting for execute to be scheduled.
        self.busy = True
        return GoalResponse.ACCEPT

    def on_accepted(self, goal):
        self.goal = goal
        self.result_future = Future()
        self.results[bytes(goal.goal_id.uuid)] = self.result_future
        self.phase = None
        self.owns_mode = False
        goal.execute()
        try:
            parameters = {key: self.get_parameter(key).value for key in self.parameter_names}
            self.routine = self.make_routine(goal.request.index, parameters)
            self.publish(Command())
            self.state_pub.publish(UInt8(data=self.mode))
        except Exception as error:
            self.get_logger().error(f'Cannot start routine: {error}')
            self.finish('aborted', restore_mode=False)

    def make_routine(self, index, parameters):
        return self.variants[index](parameters)

    async def execute(self, goal):
        # Capture the future before awaiting; a subsequent accepted goal gets
        # its own future even if this coroutine has not returned yet.
        key = bytes(goal.goal_id.uuid)
        try:
            return await self.results[key]
        finally:
            del self.results[key]

    def on_cancel(self, goal):
        return CancelResponse.ACCEPT

    def on_manual_cancel(self, message):
        if message.data == 1 and self.goal is not None:
            request = CancelGoal.Request()
            request.goal_info.goal_id = self.goal.goal_id
            self.cancel_client.call_async(request)

    def on_state(self, message):
        self.state = message.state
        if self.goal is None:
            return
        if self.state == self.mode:
            self.owns_mode = True
        elif self.owns_mode:
            # Only this node's command channels are stopped. Do not take the
            # mux back from the action or operator that displaced this goal.
            self.finish('aborted', restore_mode=False)

    def tick(self):
        if self.goal is None:
            return
        try:
            if self.goal.is_cancel_requested:
                self.finish('canceled')
                return
            if not self.owns_mode:
                return
            phase, command, done = self.routine.step(self.now(), self.positions())
            if phase != self.phase:
                self.phase = phase
                self.goal.publish_feedback(self.action_type.Feedback(phase=phase))
            self.publish(command)
            if done:
                self.finish('succeeded')
        except Exception as error:
            self.get_logger().error(f'Routine failed: {error}')
            self.finish('aborted')

    def positions(self):
        return None

    def publish(self, command):
        velocity = Twist()
        velocity.linear.x = float(command.drive)
        self.velocity_pub.publish(velocity)
        self.arm_pub.publish(Float64MultiArray(data=[
            float(command.front_arm), float(command.back_arm),
            float(command.front_drum), float(command.back_drum),
        ]))

    def finish(self, outcome, restore_mode=True):
        goal = self.goal
        if goal is None:
            return
        self.publish(Command())
        if restore_mode and self.owns_mode and self.state == self.mode:
            self.state_pub.publish(UInt8(data=RobotState.TELEOP))
        result = self.action_type.Result(success=outcome == 'succeeded')
        if outcome == 'succeeded':
            goal.succeed()
        elif outcome == 'canceled':
            goal.canceled()
        else:
            goal.abort()
        self.result_future.set_result(result)
        self.goal = None
        self.routine = None
        self.owns_mode = False
        self.busy = False


def run(node_type, args=None):
    """Run either node with serialized callbacks and cooperative execution."""
    rclpy.init(args=args)
    node = node_type()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok() and node.goal is not None:
            node.finish('aborted', restore_mode=False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
