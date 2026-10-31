"""Exercise the action protocol and mode changes through a real ROS executor."""

import time
from unittest.mock import patch

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from autonomy_msgs.action import FullAuto
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from robot_control_msgs.msg import RobotState
from std_msgs.msg import UInt8

from robot_super_intelligence.machine import Machine, STATE_TYPES
from robot_super_intelligence.outcomes import Outcome
from robot_super_intelligence.server import FullAutoServer
from robot_super_intelligence.states import RunEnd


class WaitingState:
    outcomes = frozenset({Outcome.RUN_END})

    def enter(self):
        pass

    def tick(self):
        return None

    def cancel(self):
        pass


def waiting_machine(limit):
    states = {cls: WaitingState() for cls in STATE_TYPES}
    states[RunEnd] = RunEnd()
    return Machine(limit, states)


@pytest.fixture
def ros():
    rclpy.init()
    server = FullAutoServer()
    client_node = Node('full_auto_test_client')
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    executor.add_node(client_node)
    client = ActionClient(client_node, FullAuto, 'full_auto')
    mode_pub = client_node.create_publisher(
        RobotState, 'robot_state',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    stops = []
    stop_sub = client_node.create_subscription(
        UInt8, 'robot_state/set', lambda msg: stops.append(msg.data), 10)

    def until(predicate, timeout=5):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert predicate(), 'ROS operation timed out'

    def mode(value):
        until(lambda: mode_pub.get_subscription_count() > 0)
        mode_pub.publish(RobotState(state=value))
        until(lambda: server.mode == value)

    def send(limit=5):
        future = client.send_goal_async(FullAuto.Goal(time_limit_sec=float(limit)))
        until(future.done)
        return future.result()

    assert client.wait_for_server(timeout_sec=5)
    yield server, until, mode, send, stops
    client.destroy()
    client_node.destroy_subscription(stop_sub)
    executor.shutdown()
    server.destroy_node()
    client_node.destroy_node()
    rclpy.shutdown()


def test_rejects_goal_outside_full_auto(ros):
    _, _, mode, send, _ = ros
    mode(RobotState.TELEOP)
    assert not send().accepted


@pytest.mark.parametrize('limit', [0, -1, float('nan'), float('inf')])
def test_rejects_invalid_duration(ros, limit):
    _, _, mode, send, _ = ros
    mode(RobotState.FULL_AUTO)
    assert not send(limit).accepted


def test_placeholder_reports_failure_and_requests_estop(ros):
    server, until, mode, send, stops = ros
    mode(RobotState.FULL_AUTO)
    goal = send()
    assert goal.accepted
    result = goal.get_result_async()
    until(result.done)
    until(lambda: bool(stops))
    assert result.result().status == GoalStatus.STATUS_ABORTED
    assert result.result().result.outcome == 'NOT_IMPLEMENTED'
    assert not result.result().result.success
    assert stops == [RobotState.ESTOP]
    assert not server.busy


def test_busy_rejection_cancel_and_restart(ros):
    server, until, mode, send, stops = ros
    mode(RobotState.FULL_AUTO)
    with patch('robot_super_intelligence.server.Machine', waiting_machine):
        goal = send()
        assert goal.accepted
        until(lambda: server.machine is not None)
        assert not send().accepted
        cancel = goal.cancel_goal_async()
        until(cancel.done)
        assert cancel.result().goals_canceling
        result = goal.get_result_async()
        until(result.done)
        assert result.result().status == GoalStatus.STATUS_CANCELED
        assert result.result().result.outcome == 'CANCELED'
        assert not server.busy
        next_goal = send(0.15)
        assert next_goal.accepted
        next_result = next_goal.get_result_async()
        until(next_result.done)
        assert next_result.result().status == GoalStatus.STATUS_SUCCEEDED
        assert next_result.result().result.outcome == 'TIME_LIMIT_REACHED'
        until(lambda: len(stops) == 2)
        assert stops == [RobotState.ESTOP, RobotState.ESTOP]


def test_mode_exit_interrupts_the_state(ros):
    server, until, mode, send, stops = ros
    mode(RobotState.FULL_AUTO)
    with patch('robot_super_intelligence.server.Machine', waiting_machine):
        goal = send()
        until(lambda: server.machine is not None)
        mode(RobotState.TELEOP)
        result = goal.get_result_async()
        until(result.done)
        assert result.result().status == GoalStatus.STATUS_ABORTED
        assert result.result().result.outcome == 'MODE_EXITED'
        until(lambda: bool(stops))
        assert stops == [RobotState.ESTOP]
