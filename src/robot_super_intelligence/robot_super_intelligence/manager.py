"""Public operation action and machine lifecycle for the state-manager node."""

from dataclasses import dataclass, field
import math
import time

import rclpy
from autonomy_msgs.action import ManageRobot
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from robot_control_msgs.msg import RobotState

from .arena_markers import ArenaMarkers
from .child_operation import ChildOperation
from .control_routing import ControlRouting
from .states import DumpMaterial, Excavate, Traverse
from .machine_runner import MachineRunner
from .machines import MACHINES
from .outcomes import Outcome


@dataclass
class Run:
    goal: object
    started: float = field(default_factory=time.monotonic)
    reason: str = ''
    operation: object = None
    runner: object = None
    deadline: object = None
    starting: bool = True
    completing: bool = False
    finished: bool = False
    outcome: str = ''
    state: str = ''
    latest_outcome: str = ''
    completed_cycles: int = 0


class StateManager(Node):
    def __init__(self, **kwargs):
        super().__init__('robot_super_intelligence', **kwargs)
        self.busy = False
        self.run = None
        self.arena_markers = ArenaMarkers(self)
        self.control = ControlRouting(
            self, is_busy=lambda: self.busy,
            is_stopping=lambda: self.run is not None and bool(self.run.reason),
            stop_run=self.stop_run)
        self.operations = {
            ManageRobot.Goal.NAVIGATE: Traverse(self),
            ManageRobot.Goal.DIG: Excavate(self),
            ManageRobot.Goal.DUMP: DumpMaterial(self),
        }
        self.machines = dict(MACHINES)
        self.action_server = ActionServer(
            self, ManageRobot, 'manage_robot',
            goal_callback=self.on_goal, cancel_callback=self.on_cancel,
            handle_accepted_callback=self.on_accepted)

    def stop_run(self, reason):
        # Lock first. Cancellation acknowledgement is not a stopping mechanism.
        if reason in (Outcome.ESTOP.value, Outcome.SHUTDOWN.value):
            self.control.engage_stop()
        else:
            self.control.lock_outputs()
        run = self.run
        if run is None:
            return
        if not run.reason or reason in (Outcome.ESTOP.value, Outcome.SHUTDOWN.value):
            run.reason = reason
        try:
            if run.runner is not None:
                run.runner.stop(Outcome(run.reason))
        except Exception as error:
            self.get_logger().error(f'State cleanup failed: {error}')
        finally:
            run.operation.cancel(run.reason)
            self.queue_finish(run)

    def on_goal(self, request):
        if (self.busy or self.control.changing_mode or self.control.estopped
                or not math.isfinite(request.time_limit_sec) or request.time_limit_sec <= 0):
            return GoalResponse.REJECT
        state = self.operations.get(request.operation)
        if request.operation == ManageRobot.Goal.RUN_MACHINE:
            machine = self.machines.get(request.machine_name)
            if machine is None or not machine.valid(request, self.arena_markers):
                return GoalResponse.REJECT
        elif state is None or not state.valid(request):
            return GoalResponse.REJECT
        if not self.control.mux.service_is_ready():
            return GoalResponse.REJECT
        if state is not None and not state.client.server_is_ready():
            return GoalResponse.REJECT
        self.busy = True
        return GoalResponse.ACCEPT

    def on_cancel(self, goal):
        self.stop_run(Outcome.CANCELED.value)
        return CancelResponse.ACCEPT

    def feedback(self, run, state=None):
        if state is not None:
            run.state = state
        if run.runner is not None:
            machine = run.runner.machine
            run.state = machine.current.__name__
            run.latest_outcome = machine.latest_outcome
            run.completed_cycles = machine.completed_cycles
        run.goal.publish_feedback(ManageRobot.Feedback(
            state=run.state, latest_outcome=run.latest_outcome,
            completed_cycles=run.completed_cycles,
            elapsed_sec=float(time.monotonic() - run.started)))

    def dispatch(self, run, request, on_complete):
        if self.run is run and not run.finished:
            run.operation.start(request, on_complete)

    def on_accepted(self, goal):
        run = Run(goal)
        self.run = run
        run.operation = ChildOperation(
            self, self.control, self.operations,
            on_feedback=lambda state: self.feedback(run, state),
            on_failure=self.stop_run,
            on_idle=lambda: self.queue_finish(run))
        if not goal.is_cancel_requested:
            goal.executing()
        def deadline():
            if run.deadline is not None:
                run.deadline.cancel()
            if self.run is run and not run.finished:
                self.stop_run(Outcome.TIME_LIMIT_REACHED.value)
        try:
            run.deadline = self.create_timer(
                goal.request.time_limit_sec, deadline, clock=self.control.steady_clock)
            if self.control.estopped:
                run.reason = Outcome.ESTOP.value
            if goal.is_cancel_requested:
                run.reason = Outcome.CANCELED.value
            if run.reason:
                run.starting = False
                self.stop_run(run.reason)
            elif goal.request.operation == ManageRobot.Goal.RUN_MACHINE:
                self.control.select('', lambda error: self.machine_selected(run, error))
            else:
                run.starting = False
                self.control.publish_mode(self.operations[goal.request.operation].mode)
                self.dispatch(run, goal.request, lambda outcome: self.operation_completed(run, outcome))
        except Exception as error:
            run.starting = False
            self.get_logger().error(f'Cannot start operation: {error}')
            self.stop_run(Outcome.FAILED.value)

    def machine_selected(self, run, error):
        if self.run is not run or run.finished:
            return
        run.starting = False
        if error is not None:
            self.stop_run(Outcome.FAILED.value)
        elif run.reason:
            self.queue_finish(run)
        else:
            try:
                self.control.publish_mode(RobotState.FULL_AUTO)
                run.runner = MachineRunner(
                    self, self.machines[run.goal.request.machine_name].from_goal(
                        run.goal.request, arena_markers=self.arena_markers),
                    lambda step, done: self.dispatch(run, step, done),
                    lambda: self.feedback(run))
                run.runner.start(lambda outcome: self.machine_completed(run, outcome))
            except Exception as error:
                self.get_logger().error(f'Cannot start machine: {error}')
                self.stop_run(Outcome.FAILED.value)

    def operation_completed(self, run, outcome):
        if self.run is run and not run.finished:
            run.outcome = outcome
            self.queue_finish(run)

    def machine_completed(self, run, outcome):
        if self.run is not run or run.finished:
            return
        run.outcome = outcome.value
        if not run.reason:
            self.stop_run(outcome.value)
        else:
            self.queue_finish(run)

    def queue_finish(self, run):
        # In particular, finish cancellation after the cancel callback returns
        # and ROS has moved the parent goal into CANCELING.
        self.executor.create_task(self.maybe_finish, run)

    def maybe_finish(self, run):
        if (self.run is not run or run.finished or run.completing
                or run.starting or run.operation.active):
            return
        if not run.reason and not run.outcome:
            return
        run.completing = True
        if run.deadline is not None:
            run.deadline.cancel()
            self.destroy_timer(run.deadline)
            run.deadline = None
        self.control.lock_outputs()
        if self.control.estopped:
            self.finish_run(run, None)
        else:
            self.control.select('teleop', lambda error: self.finish_run(run, error))

    def finish_run(self, run, error):
        if self.run is not run or run.finished:
            return
        if error is not None:
            if not self.control.estopped:
                self.get_logger().error(f'Cannot restore teleop: {error}')
                self.control.engage_stop()
                run.reason = Outcome.FAILED.value
        elif not self.control.estopped:
            self.control.publish_mode(RobotState.TELEOP)
        outcome = run.reason or run.outcome
        result = ManageRobot.Result(
            success=outcome == 'SUCCESS' or (
                run.runner is not None and outcome in run.runner.machine.success_outcomes),
            outcome=outcome, completed_cycles=run.completed_cycles)
        run.finished = True
        if run.goal.is_cancel_requested:
            result.success = False
            result.outcome = Outcome.CANCELED.value
            run.goal.canceled(result)
        elif result.success:
            run.goal.succeed(result)
        else:
            run.goal.abort(result)
        self.run = None
        self.busy = False

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
