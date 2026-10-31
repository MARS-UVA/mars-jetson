"""Real ROS action delegation, E-stop, ownership, and cancellation races."""

import time
import subprocess
import tempfile
from pathlib import Path
from itertools import count
from types import SimpleNamespace
from unittest.mock import patch

import pytest
import yaml
import rclpy
from action_msgs.msg import GoalStatus
from autonomy_msgs.action import Dig, Dump, ManageRobot
from geometry_msgs.msg import Twist, Vector3
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from ament_index_python.packages import get_package_prefix
from rclpy.task import Future
from robot_control_msgs.msg import RobotState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool

from robot_super_intelligence.manager import StateManager
from robot_super_intelligence.states.base import completed
from robot_super_intelligence.outcomes import Outcome


NAMESPACES = count()


class Routine(Node):
    def __init__(self, name, action_type):
        super().__init__(name + '_test')
        self.action_type = action_type
        self.goal = None
        self.result = None
        self.acceptance = None
        self.requested = False
        self.reject = False
        self.reject_cancel = False
        self.server = ActionServer(
            self, action_type, name, self.execute,
            goal_callback=self.on_goal, cancel_callback=self.on_cancel,
            handle_accepted_callback=self.accepted)

    async def on_goal(self, request):
        self.requested = True
        if self.acceptance is not None:
            await self.acceptance
        return GoalResponse.REJECT if self.reject else GoalResponse.ACCEPT

    def on_cancel(self, goal):
        return CancelResponse.REJECT if self.reject_cancel else CancelResponse.ACCEPT

    def accepted(self, goal):
        self.goal = goal
        self.result = Future(executor=self.executor)
        goal.execute()

    async def execute(self, goal):
        return await self.result

    def finish(self, success=True, canceled=False):
        result = self.action_type.Result()
        if hasattr(result, 'success'):
            result.success = success
        if canceled:
            self.goal.canceled()
        elif success:
            self.goal.succeed()
        else:
            self.goal.abort()
        self.result.set_result(result)


@pytest.fixture
def system(request):
    rclpy.init(args=['--ros-args', '-r', f'__ns:=/manager_test_{next(NAMESPACES)}'])
    executor = SingleThreadedExecutor()
    manager = StateManager(parameter_overrides=[Parameter('source_transition_sec', value=0.06)])
    client_node = Node('manager_test_client')
    routines = {ManageRobot.Goal.DIG: Routine('dig', Dig),
                ManageRobot.Goal.DUMP: Routine('dump', Dump),
                ManageRobot.Goal.NAVIGATE: Routine('navigate_to_pose', NavigateToPose)}
    nodes = [manager, client_node, *routines.values()]
    for node in nodes:
        executor.add_node(node)
    config = str(Path(__file__).resolve().parents[1] / 'config/control.yaml')
    namespace = manager.get_namespace()
    ros_args = ['--ros-args', '-r', f'__ns:={namespace}', '--params-file', config]
    logs = [tempfile.TemporaryFile(mode='w+t'), tempfile.TemporaryFile(mode='w+t')]
    twist_args = [str(Path(get_package_prefix('twist_mux')) / 'lib/twist_mux/twist_mux'),
                  *ros_args, '-r', '__node:=command_mux', '-r', 'cmd_vel_out:=cmd_vel']
    for source in ('teleop', 'navigation', 'dig', 'dump'):
        twist_args.extend(['-p', f'topics.{source}.timeout:=0.05'])
    twist_args.extend(['-p', 'topics.stop.timeout:=0.01'])
    with open(config, encoding='utf-8') as config_file:
        arm_config = yaml.safe_load(config_file)['/**/arm_drum_mux']['ros__parameters']
    arm_args = [str(Path(get_package_prefix('topic_tools')) / 'lib/topic_tools/mux'),
                arm_config['output_topic'], *arm_config['input_topics'],
                *ros_args, '-r', '__node:=arm_drum_mux']
    processes = [subprocess.Popen(command, stdout=log, stderr=log)
                 for command, log in zip((twist_args, arm_args), logs)]
    action = ActionClient(client_node, ManageRobot, 'manage_robot')
    estop = client_node.create_client(SetBool, 'robot_state/estop')
    teleop = client_node.create_client(SetBool, 'robot_state/teleop')
    velocities, arms = [], []
    client_node.create_subscription(Twist, 'cmd_vel', velocities.append, 10)
    client_node.create_subscription(Float64MultiArray, 'arm_drum_controller/commands', arms.append, 10)

    def until(predicate, timeout=5):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        if not predicate():
            for log in logs:
                log.flush()
                log.seek(0)
                print(log.read()[-3000:])
        assert predicate(), 'ROS operation timed out'

    def call(client, data):
        future = client.call_async(SetBool.Request(data=data))
        until(future.done)
        return future.result()

    def send(operation=ManageRobot.Goal.DIG, limit=10.0, **kwargs):
        goal = ManageRobot.Goal(operation=operation, time_limit_sec=float(limit), **kwargs)
        if operation == ManageRobot.Goal.NAVIGATE:
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.pose.orientation.w = 1.0
        future = action.send_goal_async(goal)
        until(future.done)
        return future.result()

    def cleanup():
        # Complete fake children left active by an assertion, so teardown is clean.
        for routine in routines.values():
            if routine.result is not None and not routine.result.done():
                routine.finish(canceled=routine.goal.is_cancel_requested)
        for process in processes:
            process.terminate()
        for process, log in zip(processes, logs):
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
            log.close()
        executor.shutdown()
        action.destroy()
        for routine in routines.values():
            routine.server.destroy()
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()

    request.addfinalizer(cleanup)
    until(lambda: action.server_is_ready() and estop.service_is_ready()
          and manager.control.service_is_ready()
          and all(state.client.server_is_ready() for state in manager.operations.values())
          )
    yield SimpleNamespace(manager=manager, routines=routines, node=client_node,
                          until=until, call=call, send=send, estop=estop, teleop=teleop,
                          velocities=velocities, arms=arms, executor=executor)



def clear(s):
    assert s.call(s.estop, False).success
    assert s.manager.mode == RobotState.IDLE
    assert s.manager.outputs_locked


def result(s, goal):
    future = goal.get_result_async()
    s.until(future.done)
    return future.result()


def test_startup_stop_and_explicit_clear_does_not_enable_teleop(system):
    s = system
    assert not s.send().accepted
    clear(s)
    assert s.manager.outputs_locked
    assert s.call(s.teleop, True).success
    assert s.manager.mode == RobotState.TELEOP
    assert not s.manager.outputs_locked
    assert s.call(s.estop, True).success
    assert s.call(s.estop, True).success
    assert s.manager.mode == RobotState.ESTOP


@pytest.mark.parametrize('operation', [ManageRobot.Goal.DIG, ManageRobot.Goal.DUMP,
                                       ManageRobot.Goal.NAVIGATE])
def test_typed_delegation_success_and_grant_release(system, operation):
    s = system
    clear(s)
    goal = s.send(operation)
    assert goal.accepted
    routine = s.routines[operation]
    s.until(lambda: routine.goal is not None)
    assert s.manager.mode == s.manager.operations[operation].mode
    routine.finish()
    response = result(s, goal)
    assert response.status == GoalStatus.STATUS_SUCCEEDED
    assert response.result.success
    assert response.result.outcome == 'SUCCESS'
    assert not s.manager.busy
    assert not s.manager.estopped
    assert s.manager.mode == RobotState.TELEOP
    assert not s.manager.outputs_locked


@pytest.mark.parametrize('limit', [0, -1, float('nan'), float('inf')])
def test_invalid_deadlines_are_rejected(system, limit):
    clear(system)
    assert not system.send(limit=limit).accepted


def test_rejection_and_failure_reach_parent(system):
    s = system
    clear(s)
    s.routines[ManageRobot.Goal.DIG].reject = True
    goal = s.send()
    assert result(s, goal).result.outcome == 'GOAL_REJECTED'
    clear(s)
    goal = s.send(ManageRobot.Goal.DUMP)
    routine = s.routines[ManageRobot.Goal.DUMP]
    s.until(lambda: routine.goal is not None)
    routine.finish(success=False)
    response = result(s, goal)
    assert response.status == GoalStatus.STATUS_ABORTED
    assert response.result.outcome == 'CHILD_FAILED'


def test_cancel_keeps_parent_reserved_until_child_is_terminal(system):
    s = system
    clear(s)
    goal = s.send()
    routine = s.routines[ManageRobot.Goal.DIG]
    s.until(lambda: s.manager.run is not None and s.manager.run.child is not None)
    cancel = goal.cancel_goal_async()
    s.until(cancel.done)
    s.until(lambda: routine.goal.is_cancel_requested)
    assert s.manager.busy
    assert not s.send().accepted
    assert not s.call(s.estop, False).success
    routine.finish(canceled=True)
    response = result(s, goal)
    assert response.status == GoalStatus.STATUS_CANCELED
    assert response.result.outcome == 'CANCELED'


def test_cancel_during_goal_acceptance_cancels_late_child(system):
    s = system
    clear(s)
    routine = s.routines[ManageRobot.Goal.DIG]
    routine.acceptance = Future(executor=s.executor)
    goal = s.send()
    s.until(lambda: routine.requested)
    cancel = goal.cancel_goal_async()
    s.until(cancel.done)
    assert s.manager.busy
    routine.acceptance.set_result(True)
    s.until(lambda: routine.goal is not None and routine.goal.is_cancel_requested)
    routine.finish(canceled=True)
    assert result(s, goal).status == GoalStatus.STATUS_CANCELED


def test_estop_blocks_commands_before_child_acknowledges_stop(system):
    s = system
    clear(s)
    goal = s.send()
    routine = s.routines[ManageRobot.Goal.DIG]
    s.until(lambda: s.manager.run is not None and s.manager.run.child is not None)
    drive = s.node.create_publisher(Twist, 'cmd_vel/dig', 1)
    arm = s.node.create_publisher(Float64MultiArray, 'arm_drum_control/dig', 1)
    s.until(lambda: drive.get_subscription_count() > 0 and arm.get_subscription_count() > 0)
    def publish_commands():
        drive.publish(Twist(linear=Vector3(x=0.2)))
        arm.publish(Float64MultiArray(data=[1.0]*4))
    stream = s.node.create_timer(0.02, publish_commands)
    s.until(lambda: s.velocities and s.velocities[-1].linear.x == 0.2
            and s.arms and list(s.arms[-1].data) == [1.0]*4)
    s.node.destroy_timer(stream)
    assert s.call(s.estop, True).success
    assert s.manager.outputs_locked
    s.until(lambda: s.velocities[-1].linear.x == 0.0 and not any(s.arms[-1].data))
    assert s.manager.busy
    s.until(lambda: routine.goal.is_cancel_requested)
    routine.finish(canceled=True)
    response = result(s, goal)
    assert response.status == GoalStatus.STATUS_ABORTED
    assert response.result.outcome == 'ESTOP'
    assert s.manager.estopped
    assert s.manager.mode == RobotState.ESTOP


def test_standalone_deadline_cancels_child_and_aborts(system):
    s = system
    clear(s)
    goal = s.send(limit=0.2)
    routine = s.routines[ManageRobot.Goal.DIG]
    s.until(lambda: routine.goal is not None and routine.goal.is_cancel_requested)
    routine.finish(canceled=True)
    response = result(s, goal)
    assert response.status == GoalStatus.STATUS_ABORTED
    assert response.result.outcome == 'TIME_LIMIT_REACHED'


def test_full_auto_placeholder_uses_internal_coordinator(system):
    s = system
    clear(s)
    goal = s.send(ManageRobot.Goal.FULL_AUTO)
    response = result(s, goal)
    assert response.status == GoalStatus.STATUS_ABORTED
    assert response.result.outcome == 'NOT_IMPLEMENTED'
    assert not any(routine.goal for routine in s.routines.values())


def test_twist_mux_lock_timeout_blocks_drive_when_manager_disappears(system):
    s = system
    clear(s)
    assert s.call(s.teleop, True).success
    drive = s.node.create_publisher(Twist, 'cmd_vel/teleop', 1)
    s.until(lambda: drive.get_subscription_count() > 0)
    stream = s.node.create_timer(0.02, lambda: drive.publish(Twist(linear=Vector3(x=0.2))))
    s.until(lambda: s.velocities and s.velocities[-1].linear.x == 0.2)
    s.node.destroy_timer(stream)
    s.manager.heartbeat_timer.cancel()
    expired = time.monotonic() + 0.6
    s.until(lambda: time.monotonic() >= expired)
    count = len(s.velocities)
    drive.publish(Twist(linear=Vector3(x=0.4)))
    settled = time.monotonic() + 0.1
    s.until(lambda: time.monotonic() >= settled)
    assert len(s.velocities) == count


def test_rejected_child_cancel_does_not_release_ownership_early(system):
    s = system
    clear(s)
    routine = s.routines[ManageRobot.Goal.DIG]
    routine.reject_cancel = True
    goal = s.send()
    s.until(lambda: s.manager.run is not None and s.manager.run.child is not None)
    cancel = goal.cancel_goal_async()
    s.until(cancel.done)
    assert s.manager.busy
    assert not s.call(s.estop, False).success
    routine.finish()
    assert result(s, goal).status == GoalStatus.STATUS_CANCELED


def test_newer_estop_supersedes_pending_clear(system):
    s = system
    pending = Future(executor=s.executor)
    async def delayed_select(*args, **kwargs):
        return await pending
    with patch.object(s.manager, 'select', delayed_select):
        clearing = s.estop.call_async(SetBool.Request(data=False))
        s.until(lambda: s.manager.changing_mode)
        assert s.call(s.estop, True).success
        pending.set_result(None)
        s.until(clearing.done)
        assert not clearing.result().success
        assert s.manager.estopped
        assert s.manager.mode == RobotState.ESTOP


def coordinator_factory(s, state):
    from robot_super_intelligence.full_auto import FullAuto
    from robot_super_intelligence.machine import Machine, STATE_TYPES
    from robot_super_intelligence.states import TraverseToConstruction
    def factory(node, limit, dispatch, feedback):
        states = {cls: cls() for cls in STATE_TYPES}
        states[TraverseToConstruction] = state
        return FullAuto(node, limit, dispatch, feedback,
                        machine_factory=lambda seconds: Machine(seconds, states))
    return factory


class WaitingState:
    outcomes = frozenset({Outcome.CONSTRUCTION_REACHED, Outcome.FAILED})
    def enter(self):
        self.cleaned = False
    def start(self, dispatch):
        self.future = Future()
        return self.future
    def cancel(self):
        self.cleaned = True


def test_full_auto_deadline_finishes_waiting_state_without_polling(system):
    s = system
    clear(s)
    state = WaitingState()
    with patch('robot_super_intelligence.manager.FullAuto', coordinator_factory(s, state)):
        goal = s.send(ManageRobot.Goal.FULL_AUTO, limit=0.2)
        response = result(s, goal)
    assert state.cleaned
    assert response.status == GoalStatus.STATUS_SUCCEEDED
    assert response.result.outcome == 'TIME_LIMIT_REACHED'


def test_full_auto_step_uses_the_internal_dispatcher_and_waits_for_cancel(system):
    s = system
    clear(s)
    class DigStep(WaitingState):
        def start(self, dispatch):
            future = Future()
            child = dispatch(ManageRobot.Goal(operation=ManageRobot.Goal.DIG,
                                              time_limit_sec=10.0))
            child.add_done_callback(lambda done: future.set_result(
                Outcome.CONSTRUCTION_REACHED if done.result() == 'SUCCESS' else Outcome.FAILED))
            return future
    state = DigStep()
    routine = s.routines[ManageRobot.Goal.DIG]
    routine.acceptance = Future(executor=s.executor)
    with patch('robot_super_intelligence.manager.FullAuto', coordinator_factory(s, state)):
        goal = s.send(ManageRobot.Goal.FULL_AUTO)
        s.until(lambda: routine.requested)
        assert s.manager.mode == RobotState.FULL_AUTO
        cancel = goal.cancel_goal_async()
        s.until(cancel.done)
        assert s.manager.busy
        routine.acceptance.set_result(True)
        s.until(lambda: routine.goal is not None and routine.goal.is_cancel_requested)
        assert s.manager.busy
        routine.finish(canceled=True)
        response = result(s, goal)
    assert state.cleaned
    assert response.status == GoalStatus.STATUS_CANCELED


def test_estop_during_teleop_handoff_is_not_cleared(system):
    s = system
    clear(s)
    goal = s.send()
    routine = s.routines[ManageRobot.Goal.DIG]
    s.until(lambda: s.manager.run is not None and s.manager.run.child is not None)
    pending = Future(executor=s.executor)
    original_select = s.manager.select
    handing_off = []
    async def delayed_select(source=''):
        if source == 'teleop':
            handing_off.append(True)
            await pending
        return await original_select(source)
    with patch.object(s.manager, 'select', delayed_select):
        routine.finish()
        s.until(lambda: bool(handing_off))
        assert s.call(s.estop, True).success
        pending.set_result(None)
        response = result(s, goal)
    assert s.manager.estopped
    assert s.manager.mode == RobotState.ESTOP
    assert response.status == GoalStatus.STATUS_ABORTED
    assert s.manager.outputs_locked


def test_cancel_during_terminal_handoff_still_restores_teleop(system):
    s = system
    clear(s)
    goal = s.send()
    routine = s.routines[ManageRobot.Goal.DIG]
    s.until(lambda: s.manager.run is not None and s.manager.run.child is not None)
    pending = Future(executor=s.executor)
    handing_off = []
    original_select = s.manager.select
    async def delayed_select(source=''):
        if source == 'teleop':
            handing_off.append(True)
            await pending
        return await original_select(source)
    with patch.object(s.manager, 'select', delayed_select):
        routine.finish()
        s.until(lambda: bool(handing_off))
        cancel = goal.cancel_goal_async()
        s.until(cancel.done)
        pending.set_result(None)
        response = result(s, goal)
    assert response.status == GoalStatus.STATUS_CANCELED
    assert s.manager.mode == RobotState.TELEOP
    assert not s.manager.estopped
    assert not s.manager.outputs_locked


@pytest.mark.parametrize('operation', [ManageRobot.Goal.DIG, ManageRobot.Goal.DUMP])
def test_state_rejects_unsupported_routine_before_dispatch(system, operation):
    s = system
    clear(s)
    assert not s.send(operation, routine_index=99).accepted
    assert s.routines[operation].goal is None
    assert not s.manager.busy
