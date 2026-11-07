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
from geometry_msgs.msg import Point, Twist, Vector3
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
from visualization_msgs.msg import Marker, MarkerArray

from robot_super_intelligence.manager import StateManager
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
        self.request = request
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
        if isinstance(operation, str):
            kwargs['machine_name'] = operation
            operation = ManageRobot.Goal.RUN_MACHINE
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
          and manager.control.mux.service_is_ready()
          and all(state.client.server_is_ready() for state in manager.operations.values())
          )
    marker_pub = client_node.create_publisher(MarkerArray, '/arena/markers', 1)
    construction = Marker(ns='Construction zone', id=4, type=Marker.LINE_STRIP)
    construction.header.frame_id = 'map'
    construction.pose.orientation.w = 1.0
    construction.points = [Point(x=x, y=y, z=0.05) for x, y in (
        (3.88, 0.0), (6.88, 0.0), (6.88, 1.5), (3.88, 1.5), (3.88, 0.0))]
    until(lambda: marker_pub.get_subscription_count() > 0)
    marker_pub.publish(MarkerArray(markers=[construction]))
    until(lambda: manager.arena_markers.construction_goal() is not None)
    yield SimpleNamespace(marker_pub=marker_pub, construction=construction,
                          manager=manager, routines=routines, node=client_node,
                          until=until, call=call, send=send, estop=estop, teleop=teleop,
                          velocities=velocities, arms=arms, executor=executor)



def clear(s):
    assert s.call(s.estop, False).success
    assert s.manager.control.mode == RobotState.IDLE
    assert s.manager.control.outputs_locked


def result(s, goal):
    future = goal.get_result_async()
    s.until(future.done)
    return future.result()


def test_startup_stop_and_explicit_clear_does_not_enable_teleop(system):
    s = system
    assert not s.send().accepted
    clear(s)
    assert s.manager.control.outputs_locked
    assert s.call(s.teleop, True).success
    assert s.manager.control.mode == RobotState.TELEOP
    assert not s.manager.control.outputs_locked
    assert s.call(s.estop, True).success
    assert s.call(s.estop, True).success
    assert s.manager.control.mode == RobotState.ESTOP


@pytest.mark.parametrize('operation', [ManageRobot.Goal.DIG, ManageRobot.Goal.DUMP,
                                       ManageRobot.Goal.NAVIGATE])
def test_typed_delegation_success_and_grant_release(system, operation):
    s = system
    clear(s)
    goal = s.send(operation)
    assert goal.accepted
    routine = s.routines[operation]
    s.until(lambda: routine.goal is not None)
    assert s.manager.control.mode == s.manager.operations[operation].mode
    routine.finish()
    response = result(s, goal)
    assert response.status == GoalStatus.STATUS_SUCCEEDED
    assert response.result.success
    assert response.result.outcome == 'SUCCESS'
    assert not s.manager.busy
    assert not s.manager.control.estopped
    assert s.manager.control.mode == RobotState.TELEOP
    assert not s.manager.control.outputs_locked


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
    s.until(lambda: s.manager.run is not None and s.manager.run.operation.goal is not None)
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
    s.until(lambda: s.manager.run is not None and s.manager.run.operation.goal is not None)
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
    assert s.manager.control.outputs_locked
    s.until(lambda: s.velocities[-1].linear.x == 0.0 and not any(s.arms[-1].data))
    assert s.manager.busy
    s.until(lambda: routine.goal.is_cancel_requested)
    routine.finish(canceled=True)
    response = result(s, goal)
    assert response.status == GoalStatus.STATUS_ABORTED
    assert response.result.outcome == 'ESTOP'
    assert s.manager.control.estopped
    assert s.manager.control.mode == RobotState.ESTOP


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


def test_full_auto_placeholder_uses_machine_runner(system):
    s = system
    clear(s)
    goal = s.send('full_auto')
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
    s.manager.control.heartbeat_timer.cancel()
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
    s.until(lambda: s.manager.run is not None and s.manager.run.operation.goal is not None)
    cancel = goal.cancel_goal_async()
    s.until(cancel.done)
    assert s.manager.busy
    assert not s.call(s.estop, False).success
    routine.finish()
    assert result(s, goal).status == GoalStatus.STATUS_CANCELED


def test_newer_estop_supersedes_pending_clear(system):
    s = system
    pending = []
    def delayed_select(source, on_complete):
        pending.append(on_complete)
    with patch.object(s.manager.control, 'select', delayed_select):
        clearing = s.estop.call_async(SetBool.Request(data=False))
        s.until(lambda: s.manager.control.changing_mode)
        assert s.call(s.estop, True).success
        pending[0](None)
        s.until(clearing.done)
        assert not clearing.result().success
        assert s.manager.control.estopped
        assert s.manager.control.mode == RobotState.ESTOP


def runner_factory(state):
    from robot_super_intelligence.machine_runner import MachineRunner
    from robot_super_intelligence.states import TraverseToConstruction
    def factory(node, machine, dispatch, feedback):
        machine.states[TraverseToConstruction] = state
        return MachineRunner(node, machine, dispatch, feedback)
    return factory


class WaitingState:
    outcomes = frozenset({Outcome.CONSTRUCTION_REACHED, Outcome.FAILED})
    def enter(self):
        self.cleaned = False
    def start(self, dispatch, on_complete):
        self.on_complete = on_complete
    def cancel(self):
        self.cleaned = True


@pytest.mark.parametrize('operation, expected_status', [
    ('full_auto', GoalStatus.STATUS_SUCCEEDED),
    ('run_start', GoalStatus.STATUS_ABORTED),
])
def test_machine_deadline_finishes_waiting_state_without_polling(system, operation, expected_status):
    s = system
    clear(s)
    state = WaitingState()
    with patch('robot_super_intelligence.manager.MachineRunner', runner_factory(state)):
        goal = s.send(operation, limit=0.2)
        response = result(s, goal)
    assert state.cleaned
    assert response.status == expected_status
    assert response.result.outcome == 'TIME_LIMIT_REACHED'


@pytest.mark.parametrize('operation', ['full_auto', 'run_start'])
def test_machine_step_uses_the_internal_dispatcher_and_waits_for_cancel(system, operation):
    s = system
    clear(s)
    class DigStep(WaitingState):
        def start(self, dispatch, on_complete):
            dispatch(ManageRobot.Goal(operation=ManageRobot.Goal.DIG, time_limit_sec=10.0),
                     lambda outcome: on_complete(Outcome.CONSTRUCTION_REACHED
                         if outcome == 'SUCCESS' else Outcome.FAILED))
    state = DigStep()
    routine = s.routines[ManageRobot.Goal.DIG]
    routine.acceptance = Future(executor=s.executor)
    with patch('robot_super_intelligence.manager.MachineRunner', runner_factory(state)):
        goal = s.send(operation)
        s.until(lambda: routine.requested)
        assert s.manager.control.mode == RobotState.FULL_AUTO
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


def test_machine_can_dispatch_successive_child_operations(system):
    s = system
    clear(s)

    class DigThenDump(WaitingState):
        def start(self, dispatch, on_complete):
            def dug(outcome):
                if outcome != 'SUCCESS':
                    on_complete(Outcome.FAILED)
                    return
                dispatch(ManageRobot.Goal(operation=ManageRobot.Goal.DUMP),
                         lambda outcome: on_complete(Outcome.CONSTRUCTION_REACHED
                             if outcome == 'SUCCESS' else Outcome.FAILED))

            dispatch(ManageRobot.Goal(operation=ManageRobot.Goal.DIG), dug)

    with patch('robot_super_intelligence.manager.MachineRunner', runner_factory(DigThenDump())):
        goal = s.send('run_start')
        dig = s.routines[ManageRobot.Goal.DIG]
        dump = s.routines[ManageRobot.Goal.DUMP]
        s.until(lambda: dig.goal is not None)
        assert dump.goal is None
        dig.finish()
        s.until(lambda: dump.goal is not None)
        assert s.manager.busy
        dump.finish()
        response = result(s, goal)

    assert response.status == GoalStatus.STATUS_SUCCEEDED
    assert response.result.outcome == 'CONSTRUCTION_REACHED'
    assert s.manager.control.mode == RobotState.TELEOP
    assert not s.manager.busy


def test_estop_during_teleop_handoff_is_not_cleared(system):
    s = system
    clear(s)
    goal = s.send()
    routine = s.routines[ManageRobot.Goal.DIG]
    s.until(lambda: s.manager.run is not None and s.manager.run.operation.goal is not None)
    pending = []
    original_select = s.manager.control.select
    handing_off = []
    def delayed_select(source, on_complete):
        if source == 'teleop':
            handing_off.append(True)
            pending.append(on_complete)
        else:
            original_select(source, on_complete)
    with patch.object(s.manager.control, 'select', delayed_select):
        routine.finish()
        s.until(lambda: bool(handing_off))
        assert s.call(s.estop, True).success
        original_select('teleop', pending[0])
        response = result(s, goal)
    assert s.manager.control.estopped
    assert s.manager.control.mode == RobotState.ESTOP
    assert response.status == GoalStatus.STATUS_ABORTED
    assert s.manager.control.outputs_locked


def test_cancel_during_terminal_handoff_still_restores_teleop(system):
    s = system
    clear(s)
    goal = s.send()
    routine = s.routines[ManageRobot.Goal.DIG]
    s.until(lambda: s.manager.run is not None and s.manager.run.operation.goal is not None)
    pending = []
    handing_off = []
    original_select = s.manager.control.select
    def delayed_select(source, on_complete):
        if source == 'teleop':
            handing_off.append(True)
            pending.append(on_complete)
        else:
            original_select(source, on_complete)
    with patch.object(s.manager.control, 'select', delayed_select):
        routine.finish()
        s.until(lambda: bool(handing_off))
        cancel = goal.cancel_goal_async()
        s.until(cancel.done)
        original_select('teleop', pending[0])
        response = result(s, goal)
    assert response.status == GoalStatus.STATUS_CANCELED
    assert s.manager.control.mode == RobotState.TELEOP
    assert not s.manager.control.estopped
    assert not s.manager.control.outputs_locked


@pytest.mark.parametrize('operation', [ManageRobot.Goal.DIG, ManageRobot.Goal.DUMP])
def test_state_rejects_unsupported_routine_before_dispatch(system, operation):
    s = system
    clear(s)
    assert not s.send(operation, routine_index=99).accepted
    assert s.routines[operation].goal is None
    assert not s.manager.busy


@pytest.mark.parametrize('operation, status, outcome', [
    ('full_auto', GoalStatus.STATUS_ABORTED, 'NOT_IMPLEMENTED'),
    ('run_start', GoalStatus.STATUS_SUCCEEDED, 'CONSTRUCTION_REACHED'),
])
def test_goal_selects_the_machine_graph(system, operation, status, outcome):
    s = system
    clear(s)
    class ReachedConstruction(WaitingState):
        def start(self, dispatch, on_complete):
            on_complete(Outcome.CONSTRUCTION_REACHED)
    with patch('robot_super_intelligence.manager.MachineRunner', runner_factory(ReachedConstruction())):
        goal = s.send(operation)
        assert goal.accepted
        response = result(s, goal)
    assert response.status == status
    assert response.result.outcome == outcome
    assert response.result.completed_cycles == 0
    assert s.manager.control.mode == RobotState.TELEOP


@pytest.mark.parametrize('operation', ['full_auto', 'run_start'])
def test_estop_interrupts_either_machine(system, operation):
    s = system
    clear(s)
    state = WaitingState()
    with patch('robot_super_intelligence.manager.MachineRunner', runner_factory(state)):
        goal = s.send(operation)
        s.until(lambda: hasattr(state, 'on_complete'))
        assert s.call(s.estop, True).success
        response = result(s, goal)
    assert state.cleaned
    assert response.status == GoalStatus.STATUS_ABORTED
    assert response.result.outcome == 'ESTOP'
    assert s.manager.control.mode == RobotState.ESTOP


def test_unknown_operation_is_rejected(system):
    clear(system)
    assert not system.send(operation=99).accepted
    assert not system.manager.busy


@pytest.mark.parametrize('marker_shift', [0.0, 0.5])
def test_dig_traverse_dump_delegates_in_order_and_finishes(system, marker_shift):
    s = system
    clear(s)
    goal = s.send('dig_traverse_dump')
    assert goal.accepted
    dig, nav, dump = [s.routines[operation] for operation in (
        ManageRobot.Goal.DIG, ManageRobot.Goal.NAVIGATE, ManageRobot.Goal.DUMP)]
    s.until(lambda: dig.goal is not None)
    assert not nav.requested and not dump.requested
    assert dig.request.index == 0
    # Read the latest published geometry after digging, not at parent acceptance.
    s.construction.pose.position.x = marker_shift
    s.marker_pub.publish(MarkerArray(markers=[s.construction]))
    s.until(lambda: s.manager.arena_markers.construction_goal().pose.position.x
            == pytest.approx(5.38 + marker_shift))
    dig.finish()
    s.until(lambda: nav.goal is not None)
    assert not dump.requested
    assert nav.request.pose.header.frame_id == 'map'
    assert nav.request.pose.pose.position.x == pytest.approx(5.38 + marker_shift)
    assert nav.request.pose.pose.position.y == 1.5
    assert nav.request.pose.pose.orientation.z == pytest.approx(-2**-0.5)
    assert nav.request.pose.pose.orientation.w == pytest.approx(2**-0.5)
    nav.finish()
    s.until(lambda: dump.goal is not None)
    assert dump.request.index == 0
    dump.finish()
    response = result(s, goal)
    assert response.status == GoalStatus.STATUS_SUCCEEDED
    assert response.result.outcome == 'DUMP_SUCCESS'
    assert response.result.completed_cycles == 1
    assert not s.manager.busy
    assert s.manager.control.mode == RobotState.TELEOP


@pytest.mark.parametrize('stage', [0, 1, 2])
@pytest.mark.parametrize('stop', ['failure', 'cancel', 'estop'])
def test_dig_traverse_dump_stops_at_each_stage(system, stage, stop):
    s = system
    clear(s)
    goal = s.send('dig_traverse_dump')
    routines = [s.routines[operation] for operation in (
        ManageRobot.Goal.DIG, ManageRobot.Goal.NAVIGATE, ManageRobot.Goal.DUMP)]
    for routine in routines[:stage]:
        s.until(lambda: routine.goal is not None)
        routine.finish()
    routine = routines[stage]
    s.until(lambda: routine.goal is not None)
    if stop == 'failure':
        routine.finish(success=False)
    else:
        if stop == 'cancel':
            cancel = goal.cancel_goal_async()
            s.until(cancel.done)
        else:
            assert s.call(s.estop, True).success
        s.until(lambda: routine.goal.is_cancel_requested)
        assert s.manager.busy
        routine.finish(canceled=True)
    response = result(s, goal)
    assert response.status == (GoalStatus.STATUS_CANCELED if stop == 'cancel'
                               else GoalStatus.STATUS_ABORTED)
    assert response.result.outcome == {'failure': 'FAILED', 'cancel': 'CANCELED',
                                       'estop': 'ESTOP'}[stop]
    assert response.result.completed_cycles == 0
    assert not any(r.requested for r in routines[stage+1:])
    assert s.manager.control.mode == (RobotState.ESTOP if stop == 'estop' else RobotState.TELEOP)


def test_dig_traverse_dump_rejects_invalid_routine_before_dig(system):
    s = system
    clear(s)
    assert not s.send('dig_traverse_dump', routine_index=99).accepted
    assert not any(r.requested for r in s.routines.values())


def test_dig_traverse_dump_deadline_aborts_and_waits_for_child(system):
    s = system
    clear(s)
    goal = s.send('dig_traverse_dump', limit=0.2)
    dig = s.routines[ManageRobot.Goal.DIG]
    s.until(lambda: dig.goal is not None and dig.goal.is_cancel_requested)
    assert s.manager.busy
    dig.finish(canceled=True)
    response = result(s, goal)
    assert response.status == GoalStatus.STATUS_ABORTED
    assert response.result.outcome == 'TIME_LIMIT_REACHED'
    assert not s.routines[ManageRobot.Goal.NAVIGATE].requested
    assert not s.routines[ManageRobot.Goal.DUMP].requested


def test_dig_traverse_dump_requires_published_construction(system):
    s = system
    clear(s)
    s.marker_pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
    s.until(lambda: s.manager.arena_markers.construction_goal() is None)
    assert not s.send('dig_traverse_dump').accepted
    assert not any(r.requested for r in s.routines.values())


def test_deleted_construction_during_dig_aborts_before_navigation(system):
    s = system
    clear(s)
    goal = s.send('dig_traverse_dump')
    dig = s.routines[ManageRobot.Goal.DIG]
    s.until(lambda: dig.goal is not None)
    s.marker_pub.publish(MarkerArray(markers=[Marker(
        ns='Construction zone', id=4, action=Marker.DELETE)]))
    s.until(lambda: s.manager.arena_markers.construction_goal() is None)
    dig.finish()
    response = result(s, goal)
    assert response.status == GoalStatus.STATUS_ABORTED
    assert response.result.outcome == 'FAILED'
    assert not s.routines[ManageRobot.Goal.NAVIGATE].requested
    assert not s.routines[ManageRobot.Goal.DUMP].requested


@pytest.mark.parametrize('name', ['', 'missing_machine', 'full_auto '])
def test_empty_or_unknown_machine_name_is_rejected(system, name):
    s = system
    clear(s)
    assert not s.send(ManageRobot.Goal.RUN_MACHINE, machine_name=name).accepted
    assert not s.manager.busy
    assert not any(r.requested for r in s.routines.values())


@pytest.mark.parametrize('operation', [4, 5])
def test_old_machine_operation_numbers_are_rejected(system, operation):
    s = system
    clear(s)
    assert not s.send(operation).accepted
    assert not s.manager.busy


def test_machine_can_be_registered_under_a_new_name_without_interface_changes(system):
    from robot_super_intelligence.machines import RunStartMachine
    s = system
    clear(s)
    s.manager.machines['club_test'] = RunStartMachine

    class ReachedConstruction(WaitingState):
        def start(self, dispatch, on_complete):
            on_complete(Outcome.CONSTRUCTION_REACHED)

    with patch('robot_super_intelligence.manager.MachineRunner', runner_factory(ReachedConstruction())):
        goal = s.send(ManageRobot.Goal.RUN_MACHINE, machine_name='club_test')
        assert goal.accepted
        response = result(s, goal)
    assert response.status == GoalStatus.STATUS_SUCCEEDED
    assert response.result.outcome == 'CONSTRUCTION_REACHED'
