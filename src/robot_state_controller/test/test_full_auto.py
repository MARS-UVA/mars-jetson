"""Full-auto routing and idempotent stop requests."""

from types import SimpleNamespace
from unittest.mock import Mock

from robot_control_msgs.msg import RobotState
from std_msgs.msg import UInt8

from robot_state_controller.robot_state_controller import RobotStateControllerNode


def test_full_auto_selects_both_dedicated_channels():
    node = SimpleNamespace(
        state=RobotState.FULL_AUTO, send_mux_request=Mock(),
        cmd_vel_mux_client_=object(), arm_drum_mux_client_=object(),
        robot_state_publisher_=Mock())
    RobotStateControllerNode.control_robot_state(node)
    assert node.send_mux_request.call_args_list[0].args == (
        'cmd_vel/full_auto', node.cmd_vel_mux_client_)
    assert node.send_mux_request.call_args_list[1].args == (
        'arm_drum_control/full_auto', node.arm_drum_mux_client_)
    assert node.robot_state_publisher_.publish.call_args.args[0].state == RobotState.FULL_AUTO


def test_repeated_set_estop_never_toggles_to_teleop():
    node = SimpleNamespace(state=RobotState.FULL_AUTO, control_robot_state=Mock())
    for _ in range(2):
        RobotStateControllerNode.robot_state_set_callback(node, UInt8(data=RobotState.ESTOP))
        assert node.state == RobotState.ESTOP
    assert node.control_robot_state.call_count == 2


def test_unknown_set_mode_is_ignored():
    node = SimpleNamespace(state=RobotState.ESTOP, control_robot_state=Mock(),
                           get_logger=Mock())
    RobotStateControllerNode.robot_state_set_callback(node, UInt8(data=255))
    assert node.state == RobotState.ESTOP
    node.control_robot_state.assert_not_called()
