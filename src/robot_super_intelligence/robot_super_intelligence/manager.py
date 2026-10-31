"""Public operation action, command ownership, and E-stop lifecycle."""

from dataclasses import dataclass, field
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from autonomy_msgs.action import ManageRobot
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.clock import Clock, ClockType
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from robot_control_msgs.msg import RobotState
from geometry_msgs.msg import Twist
from topic_tools_interfaces.srv import MuxSelect
from rclpy.task import Future
from std_msgs.msg import Bool, Float64MultiArray, UInt8
from std_srvs.srv import SetBool

from .states import DumpMaterial, Excavate, Traverse
from .full_auto import FullAuto
from .outcomes import Outcome


@dataclass
class Run:
    goal: object
    started: float = field(default_factory=time.monotonic)
    reason: str = ''
    child: object = None
    cancel_sent: bool = False
    dispatching: bool = False
    coordinator: object = None
    deadline: object = None
    steps: list = field(default_factory=list)
    state: str = ''
    latest_outcome: str = ''
    completed_cycles: int = 0


class StateManager(Node):
    def __init__(self, **kwargs):
        super().__init__('robot_super_intelligence', **kwargs)
        self.busy = False
        self.estopped = True
        self.mode = RobotState.ESTOP
        self.run = None
        self.safety_epoch = 0
        self.changing_mode = False
        self.services_group = ReentrantCallbackGroup()
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(RobotState, 'robot_state', latched)
        lock_topic = self.declare_parameter('stop_lock_topic', 'control/locks/stop').value
        self.lock_pub = self.create_publisher(Bool, lock_topic, latched)
        self.outputs_locked = True
        self.stop_future = None
        self.transition_sec = self.declare_parameter('source_transition_sec', 0.5).value
        self.arm_stop_topic = self.declare_parameter('arm_stop_topic', 'arm_drum_control/stop').value
        self.arm_topics = {source: self.declare_parameter(
            f'arm_topics.{source}', f'arm_drum_control/{source}').value
            for source in ('teleop', 'dig', 'dump')}
        stop_topic = self.declare_parameter('stop_velocity_topic', 'cmd_vel/stop').value
        self.stop_velocity_pub = self.create_publisher(Twist, stop_topic, 1)
        self.stop_arm_pub = self.create_publisher(Float64MultiArray, self.arm_stop_topic, 1)
        self.control = self.create_client(MuxSelect, 'arm_drum_mux/select')
        self.operations = {
            ManageRobot.Goal.NAVIGATE: Traverse(self),
            ManageRobot.Goal.DIG: Excavate(self),
            ManageRobot.Goal.DUMP: DumpMaterial(self),
        }
        self.action_server = ActionServer(
            self, ManageRobot, 'manage_robot', self.execute,
            goal_callback=self.on_goal, cancel_callback=self.on_cancel)
        self.estop_service = self.create_service(SetBool, 'robot_state/estop', self.on_estop, callback_group=self.services_group)
        self.teleop_service = self.create_service(SetBool, 'robot_state/teleop', self.on_teleop, callback_group=self.services_group)
        # Legacy mode inputs can request teleop/idle/stop, never start a routine.
        self.set_sub = self.create_subscription(UInt8, 'robot_state/set', self.on_mode, 1)
        self.toggle_sub = self.create_subscription(UInt8, 'robot_state/toggle', self.on_mode, 1)
        self.cancel_sub = self.create_subscription(UInt8, 'cancel_command', self.on_manual_cancel, 1)
        self.heartbeat_timer = self.create_timer(
            self.declare_parameter('heartbeat_period_sec', 0.1).value,
            self.heartbeat, clock=self.steady_clock)
        self.lock_pub.publish(Bool(data=True))
        self.publish_mode(RobotState.ESTOP)
        self.heartbeat()

    def heartbeat(self):
        self.lock_pub.publish(Bool(data=self.outputs_locked))
        if self.outputs_locked:
            self.stop_velocity_pub.publish(Twist())
            self.stop_arm_pub.publish(Float64MultiArray(data=[0.0] * 4))

    def publish_mode(self, mode):
        self.mode = mode
        self.state_pub.publish(RobotState(state=mode))

    def hold_outputs(self):
        self.outputs_locked = True
        self.heartbeat()
        if self.control.service_is_ready():
            self.stop_future = self.control.call_async(MuxSelect.Request(topic=self.arm_stop_topic))
        else:
            self.stop_future = None

    async def select(self, source=''):
        if not self.control.service_is_ready():
            raise RuntimeError('Arm/drum mux unavailable')
        if source not in ('', 'teleop', 'navigation', 'dig', 'dump'):
            raise ValueError(f'Unknown operation source: {source}')
        self.hold_outputs()
        response = await self.stop_future
        if not response.success:
            raise RuntimeError('Arm/drum mux rejected the stop input')
        if not source:
            return
        # Drain previous velocity inputs while both outputs remain stopped.
        if self.transition_sec > 0:
            future = Future(executor=self.executor)
            def elapsed():
                timer.cancel()
                future.set_result(None)
            timer = self.create_timer(self.transition_sec, elapsed, clock=self.steady_clock)
            try:
                await future
            finally:
                self.destroy_timer(timer)
        if self.estopped or (self.run and self.run.reason and source != 'teleop'):
            raise RuntimeError('Operation interrupted during source transition')
        topic = self.arm_topics.get(source, self.arm_stop_topic)
        response = await self.control.call_async(MuxSelect.Request(topic=topic))
        if not response.success:
            raise RuntimeError(f'Arm/drum mux rejected {topic}')
        if self.estopped or (self.run and self.run.reason and source != 'teleop'):
            self.hold_outputs()
            raise RuntimeError('Operation interrupted during source transition')
        self.outputs_locked = False
        self.heartbeat()

    def lock_outputs(self):
        self.safety_epoch += 1
        self.hold_outputs()

    def engage_stop(self):
        self.lock_outputs()
        self.estopped = True
        self.publish_mode(RobotState.ESTOP)

    def stop_run(self, reason):
        # Lock first. Cancellation acknowledgement is not a stopping mechanism.
        if reason in (Outcome.ESTOP.value, Outcome.SHUTDOWN.value):
            self.engage_stop()
        else:
            self.lock_outputs()
        run = self.run
        if run is None:
            return
        if not run.reason or reason in (Outcome.ESTOP.value, Outcome.SHUTDOWN.value):
            run.reason = reason
        try:
            if run.coordinator is not None:
                run.coordinator.stop(Outcome(run.reason))
        except Exception as error:
            self.get_logger().error(f'State cleanup failed: {error}')
        finally:
            self.cancel_child(run)

    def cancel_child(self, run):
        if run.child is not None and not run.cancel_sent:
            run.cancel_sent = True
            future = run.child.cancel_goal_async()
            future.add_done_callback(self.cancel_response)

    def cancel_response(self, future):
        try:
            response = future.result()
            if not response.goals_canceling:
                self.get_logger().warn('Child cancellation rejected or already terminal; waiting for result')
        except Exception as error:
            self.get_logger().error(f'Child cancellation failed: {error}; command lock remains active')

    async def on_estop(self, request, response):
        if request.data:
            self.stop_run('ESTOP')
            if self.stop_future is None:
                response.message = 'Drive stopped, but arm/drum mux is unavailable'
                return response
            stopped = await self.stop_future
            if not stopped.success:
                response.message = 'Arm/drum mux rejected the stop input'
                return response
            response.success = True
            response.message = 'E-stop engaged; active work is being canceled'
            return response
        if self.busy or self.changing_mode:
            response.message = 'Waiting for the active operation to stop'
            return response
        self.changing_mode = True
        epoch = self.safety_epoch
        try:
            await self.select()
            if epoch != self.safety_epoch:
                response.message = 'A newer stop request superseded this clear'
                return response
            self.estopped = False
            self.publish_mode(RobotState.IDLE)
            response.success = True
            response.message = 'E-stop cleared; robot is idle'
        except Exception as error:
            response.message = str(error)
        finally:
            self.changing_mode = False
        return response

    async def on_teleop(self, request, response):
        if self.estopped or self.busy or self.changing_mode:
            response.message = 'Clear E-stop and finish the active operation first'
            return response
        self.changing_mode = True
        epoch = self.safety_epoch
        try:
            await self.select('teleop' if request.data else '')
            if epoch != self.safety_epoch:
                response.message = 'A newer stop request superseded this mode change'
                return response
            self.publish_mode(RobotState.TELEOP if request.data else RobotState.IDLE)
            response.success = True
        except Exception as error:
            self.engage_stop()
            response.message = str(error)
        finally:
            self.changing_mode = False
        return response

    def on_mode(self, message):
        if message.data == RobotState.ESTOP:
            self.stop_run('ESTOP')
        elif message.data in (RobotState.TELEOP, RobotState.IDLE):
            if self.busy:
                self.stop_run(Outcome.MODE_EXITED.value)
            else:
                self.executor.create_task(self.on_teleop,
                    SetBool.Request(data=message.data == RobotState.TELEOP), SetBool.Response())
        else:
            self.get_logger().warn('Start autonomous modes through /manage_robot, not mode topics')

    def on_manual_cancel(self, message):
        if message.data == 1:
            # Legacy topic acts as a stop; only the action cancel service can
            # produce the protocol's CANCELED status.
            self.stop_run(Outcome.MODE_EXITED.value)

    def on_goal(self, request):
        if (self.busy or self.changing_mode or self.estopped
                or not math.isfinite(request.time_limit_sec) or request.time_limit_sec <= 0):
            return GoalResponse.REJECT
        state = self.operations.get(request.operation)
        if request.operation != ManageRobot.Goal.FULL_AUTO and (state is None or not state.valid(request)):
            return GoalResponse.REJECT
        if not self.control.service_is_ready():
            return GoalResponse.REJECT
        if state is not None and not state.client.server_is_ready():
            return GoalResponse.REJECT
        self.busy = True
        return GoalResponse.ACCEPT

    def on_cancel(self, goal):
        self.stop_run(Outcome.CANCELED.value)
        return CancelResponse.ACCEPT

    def feedback(self, run):
        if run.coordinator is not None:
            machine = run.coordinator.machine
            run.state = machine.current.__name__
            run.latest_outcome = machine.latest_outcome
            run.completed_cycles = machine.completed_cycles
        run.goal.publish_feedback(ManageRobot.Feedback(
            state=run.state, latest_outcome=run.latest_outcome,
            completed_cycles=run.completed_cycles,
            elapsed_sec=float(time.monotonic() - run.started)))

    def dispatch_step(self, run, request):
        task = self.executor.create_task(self.dispatch, run, request)
        run.steps.append(task)
        return task

    async def dispatch(self, run, request):
        """Serialize steps and revoke their grants before the next transition."""
        if run.dispatching:
            return 'CHILD_BUSY'
        run.dispatching = True
        try:
            return await self.dispatch_child(run, request)
        finally:
            try:
                await self.select()
            finally:
                run.dispatching = False

    async def dispatch_child(self, run, request):
        """Shared by standalone goals and full-auto steps, without recursion."""
        if run.reason:
            return run.reason
        state = self.operations.get(request.operation)
        if state is None or not state.client.server_is_ready():
            return 'SERVER_UNAVAILABLE'
        if not state.valid(request):
            return 'INVALID_GOAL'
        source = state.source
        await self.select(source)
        if run.reason:
            return run.reason
        run.state = source
        self.feedback(run)
        def child_feedback(message):
            if self.run is run:
                run.state = getattr(message.feedback, 'phase', source)
                self.feedback(run)
        # Keep the run reserved while awaiting acceptance. A late-accepted goal
        # is canceled below if a stop arrived during this await.
        child = await state.send_goal(request, child_feedback)
        if not child.accepted:
            return 'GOAL_REJECTED'
        run.child = child
        run.cancel_sent = False
        if run.reason:
            self.cancel_child(run)
        result = await child.get_result_async()
        run.child = None
        if run.reason:
            return run.reason
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            return 'CHILD_FAILED'
        if hasattr(result.result, 'success') and not result.result.success:
            return 'CHILD_FAILED'
        return 'SUCCESS'

    async def execute(self, goal):
        run = Run(goal)
        self.run = run
        request = goal.request
        def deadline():
            run.deadline.cancel()
            if self.run is run:
                self.stop_run(Outcome.TIME_LIMIT_REACHED.value)
        run.deadline = self.create_timer(request.time_limit_sec, deadline, clock=self.steady_clock)
        outcome = Outcome.FAILED.value
        try:
            if self.estopped:
                run.reason = 'ESTOP'
            if goal.is_cancel_requested:
                self.stop_run(Outcome.CANCELED.value)
            if not run.reason and request.operation == ManageRobot.Goal.FULL_AUTO:
                await self.select()
                if not run.reason:
                    self.publish_mode(RobotState.FULL_AUTO)
                    run.coordinator = FullAuto(
                        self, request.time_limit_sec,
                        lambda step: self.dispatch_step(run, step),
                        lambda: self.feedback(run))
                    outcome = (await run.coordinator.start()).value
                    self.stop_run(outcome)
                    # Include goals still awaiting acceptance when a stop arrived.
                    for task in run.steps:
                        await task
            elif not run.reason:
                self.publish_mode(self.operations[request.operation].mode)
                outcome = await self.dispatch(run, request)
            outcome = run.reason or outcome
        except Exception as error:
            self.get_logger().error(f'Operation failed: {error}')
            self.stop_run(Outcome.FAILED.value)
            if run.child is not None:
                await run.child.get_result_async()
            outcome = run.reason or Outcome.FAILED.value
        finally:
            run.deadline.cancel()
            self.destroy_timer(run.deadline)
            # A normal terminal result hands control back to teleop. Only an
            # explicit E-stop stays latched; restoring a grant must not clear it.
            self.lock_outputs()
            if not self.estopped:
                try:
                    await self.select('teleop')
                    if not self.estopped:
                        self.publish_mode(RobotState.TELEOP)
                except Exception as error:
                    if not self.estopped:
                        self.get_logger().error(f'Cannot restore teleop: {error}')
                        self.engage_stop()
                        run.reason = Outcome.FAILED.value
            outcome = run.reason or outcome
            result = ManageRobot.Result(
                success=outcome == 'SUCCESS' or (
                    request.operation == ManageRobot.Goal.FULL_AUTO
                    and outcome == Outcome.TIME_LIMIT_REACHED.value),
                outcome=outcome, completed_cycles=run.completed_cycles)
            if goal.is_cancel_requested:
                result.success = False
                result.outcome = Outcome.CANCELED.value
                goal.canceled()
            elif result.success:
                goal.succeed()
            else:
                goal.abort()
            self.run = None
            self.busy = False
        return result

    def destroy_node(self):
        self.action_server.destroy()
        for state in self.operations.values():
            state.client.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StateManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop_run(Outcome.SHUTDOWN.value)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
