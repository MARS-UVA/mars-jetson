"""Inactive teleop must not compete with autonomous twist_mux inputs."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from robot_control_msgs.msg import RobotState

from teleop.teleop import TeleopNode


@pytest.mark.parametrize('state', [RobotState.DIG, RobotState.DUMP, RobotState.NAVIGATION,
                                   RobotState.FULL_AUTO, RobotState.ESTOP, RobotState.IDLE])
def test_inactive_teleop_does_not_publish_stop_commands(state):
    node = SimpleNamespace(robot_state=RobotState(state=state),
                           _cmd_vel_publisher=Mock(), _arm_drum_control_publisher=Mock())
    TeleopNode._TeleopNode__stopped_motors(node)
    node._cmd_vel_publisher.publish.assert_not_called()
    node._arm_drum_control_publisher.publish.assert_not_called()


def test_active_teleop_can_publish_stop_commands():
    node = SimpleNamespace(robot_state=RobotState(state=RobotState.TELEOP),
                           _cmd_vel_publisher=Mock(), _arm_drum_control_publisher=Mock())
    TeleopNode._TeleopNode__stopped_motors(node)
    assert node._cmd_vel_publisher.publish.call_args.args[0].linear.x == 0.0
    assert list(node._arm_drum_control_publisher.publish.call_args.args[0].data) == [0.0] * 4
