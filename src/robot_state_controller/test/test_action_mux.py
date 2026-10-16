"""Check that the robot state selects both channels for the same action."""

from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call

from robot_state_controller.robot_state_controller import (
    DIG_MODE, DUMP_MODE, ESTOP_MODE, NAVIGATION_MODE, TELEOP_MODE,
    RobotStateControllerNode,
)


class ActionMuxTests(unittest.TestCase):
    def test_state_routes_both_muxes(self):
        for state, velocity, arms in [
            (DIG_MODE, 'cmd_vel/dig', 'arm_drum_control/dig'),
            (DUMP_MODE, 'cmd_vel/dump', 'arm_drum_control/dump'),
            (TELEOP_MODE, 'cmd_vel/teleop', 'arm_drum_control/teleop'),
            (ESTOP_MODE, 'cmd_vel/teleop', 'arm_drum_control/teleop'),
            (NAVIGATION_MODE, 'cmd_vel/navigation', 'arm_drum_control/stop'),
        ]:
            with self.subTest(state=state):
                node = SimpleNamespace(
                    state=state, send_mux_request=Mock(),
                    cmd_vel_mux_client_=Mock(), arm_drum_mux_client_=Mock(),
                    robot_state_publisher_=Mock())
                RobotStateControllerNode.control_robot_state(node)
                self.assertEqual(node.send_mux_request.call_args_list, [
                    call(velocity, node.cmd_vel_mux_client_),
                    call(arms, node.arm_drum_mux_client_),
                ])
                self.assertEqual(
                    node.robot_state_publisher_.publish.call_args.args[0].state, state)


if __name__ == '__main__':
    unittest.main()
