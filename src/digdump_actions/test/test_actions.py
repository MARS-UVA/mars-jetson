"""Unit tests for Bruno sequences and action lifecycle decisions."""

from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from autonomy_msgs.action import Dig, Dump
from rclpy.action import GoalResponse
from robot_control_msgs.msg import RobotState

from digdump_actions.dig import BrunoDig, DEFAULTS as DIG_DEFAULTS, DigNode
from digdump_actions.dump import BrunoDump, DEFAULTS as DUMP_DEFAULTS, DumpNode
from digdump_actions.server import Command, RoutineServer


class DigTests(unittest.TestCase):
    def setUp(self):
        self.routine = BrunoDig(dict(DIG_DEFAULTS))

    def test_waits_for_real_feedback(self):
        self.assertEqual(self.routine.step(0, None), ('lowering', Command(), False))

    def test_each_arm_slows_and_stops_independently(self):
        _, command, done = self.routine.step(0, (0.5, 0.8))
        self.assertEqual((command.front_arm, command.back_arm), (-1.0, -0.75))
        self.assertEqual((command.front_drum, command.back_drum), (-0.75, -0.75))
        self.assertFalse(done)
        _, command, _ = self.routine.step(1, (0.95, 0.8))
        self.assertEqual((command.front_arm, command.back_arm), (0.0, -0.75))
        self.assertEqual(command.drive, 0)

    def test_phase_order_timing_and_independent_raising(self):
        self.assertEqual(self.routine.step(0, (0.9, 0.9)), ('digging', Command(), False))
        phase, command, _ = self.routine.step(0.1, (0.9, 0.9))
        self.assertEqual(phase, 'digging')
        self.assertEqual(command, Command(front_drum=0.75, back_drum=0.75))
        self.assertEqual(self.routine.step(3.0, (0.9, 0.9))[0], 'digging')
        self.assertEqual(self.routine.step(3.2, (0.9, 0.9)), ('raising', Command(), False))
        phase, command, done = self.routine.step(3.3, (0.6, 0.8))
        self.assertEqual(phase, 'raising')
        self.assertEqual(command, Command(back_arm=1.0))
        self.assertFalse(done)
        self.assertEqual(self.routine.step(4, (0.59, 0.6)), ('raising', Command(), True))

    def test_configurable_raise_target(self):
        self.routine.p['actuator_raise_target'] = 0.7
        self.routine.phase = 'raising'
        self.assertTrue(self.routine.step(0, (0.7, 0.65))[2])

    def test_nonfinite_feedback_is_not_used(self):
        node = SimpleNamespace(actuator_positions=None)
        DigNode.on_position(node, SimpleNamespace(
            front_actuator_position=0.4, back_actuator_position=0.5))
        self.assertEqual(node.actuator_positions, (0.4, 0.5))
        DigNode.on_position(node, SimpleNamespace(
            front_actuator_position=float('nan'), back_actuator_position=0.5))
        self.assertIsNone(node.actuator_positions)


class DumpTests(unittest.TestCase):
    def test_selection_priority_direction_and_drum(self):
        for front, back in [(0, 0), (1, 0), (0, 1), (1, 1)]:
            with self.subTest(front=front, back=back):
                node = SimpleNamespace(back_selected=False)
                DumpNode.on_selection(node, SimpleNamespace(
                    front_arm_control=front, back_arm_control=back))
                routine = BrunoDump(dict(DUMP_DEFAULTS), node.back_selected)
                self.assertEqual(routine.step(0)[1].drive, -0.25 if back else 0.25)
                self.assertEqual(routine.step(6), ('dumping', Command(), False))
                phase, command, _ = routine.step(6.1)
                self.assertEqual(phase, 'dumping')
                self.assertEqual(command, Command(
                    back_drum=-1.0) if back else Command(front_drum=-1.0))
                self.assertFalse(routine.step(12.0)[2])
                self.assertEqual(routine.step(12.2), ('dumping', Command(), True))

    def test_selection_and_parameters_are_snapshotted(self):
        node = SimpleNamespace(back_selected=False, variants={0: BrunoDump})
        parameters = dict(DUMP_DEFAULTS, move_time=2.0, dump_time=1.0)
        routine = DumpNode.make_routine(node, 0, parameters)
        DumpNode.on_selection(node, SimpleNamespace(back_arm_control=1))
        self.assertEqual(routine.step(0)[1].drive, 0.25)
        self.assertEqual(routine.step(2)[0], 'dumping')
        self.assertEqual(routine.step(2.1)[1], Command(front_drum=-1.0))
        self.assertTrue(routine.step(3.2)[2])


class LifecycleTests(unittest.TestCase):
    def test_both_actions_expose_bruno_variant_zero(self):
        for action in [Dig, Dump]:
            self.assertEqual(action.Goal.BRUNO_INDEX, 0)
            self.assertEqual(action.Goal().index, 0)
            self.assertEqual(action.Feedback(phase='lowering').phase, 'lowering')
            self.assertTrue(action.Result(success=True).success)

    def test_unsupported_and_busy_goals_are_rejected(self):
        node = SimpleNamespace(busy=False, variants={0: Mock()})
        self.assertEqual(RoutineServer.on_goal(node, SimpleNamespace(index=1)),
                         GoalResponse.REJECT)
        self.assertFalse(node.busy)
        self.assertEqual(RoutineServer.on_goal(node, SimpleNamespace(index=0)),
                         GoalResponse.ACCEPT)
        self.assertEqual(RoutineServer.on_goal(node, SimpleNamespace(index=0)),
                         GoalResponse.REJECT)

    def test_displacement_aborts_without_restoring_teleop(self):
        node = SimpleNamespace(goal=Mock(), mode=RobotState.DIG, owns_mode=False,
                               finish=Mock())
        RoutineServer.on_state(node, SimpleNamespace(state=RobotState.DIG))
        self.assertTrue(node.owns_mode)
        RoutineServer.on_state(node, SimpleNamespace(state=RobotState.DUMP))
        node.finish.assert_called_once_with('aborted', restore_mode=False)

    def test_waits_for_mode_before_advancing_and_honors_cancel(self):
        node = SimpleNamespace(goal=SimpleNamespace(is_cancel_requested=False),
                               owns_mode=False, routine=Mock(), finish=Mock())
        RoutineServer.tick(node)
        node.routine.step.assert_not_called()
        node.goal.is_cancel_requested = True
        RoutineServer.tick(node)
        node.finish.assert_called_once_with('canceled')

    def test_manual_cancel_uses_standard_action_cancel_service(self):
        from unique_identifier_msgs.msg import UUID
        node = SimpleNamespace(
            goal=SimpleNamespace(goal_id=UUID(uuid=list(range(16)))), cancel_client=Mock())
        RoutineServer.on_manual_cancel(node, SimpleNamespace(data=1))
        request = node.cancel_client.call_async.call_args.args[0]
        self.assertEqual(request.goal_info.goal_id, node.goal.goal_id)

    def test_tick_publishes_phase_changes_and_completes(self):
        node = SimpleNamespace(
            goal=Mock(is_cancel_requested=False), owns_mode=True, now=lambda: 1,
            positions=lambda: None, routine=Mock(), phase=None,
            action_type=Dig, publish=Mock(), finish=Mock())
        node.routine.step.return_value = ('digging', Command(front_drum=0.75), False)
        RoutineServer.tick(node)
        RoutineServer.tick(node)
        node.goal.publish_feedback.assert_called_once()
        self.assertEqual(node.goal.publish_feedback.call_args.args[0].phase, 'digging')
        node.routine.step.return_value = ('raising', Command(), True)
        RoutineServer.tick(node)
        self.assertEqual(node.goal.publish_feedback.call_count, 2)
        node.finish.assert_called_once_with('succeeded')

    def test_execution_error_aborts(self):
        node = SimpleNamespace(
            goal=Mock(is_cancel_requested=False), owns_mode=True, now=lambda: 1,
            positions=lambda: None, routine=Mock(), finish=Mock(), get_logger=Mock())
        node.routine.step.side_effect = RuntimeError('failed')
        RoutineServer.tick(node)
        node.finish.assert_called_once_with('aborted')

    def test_accepted_goal_snapshots_parameters_and_requests_its_mode(self):
        parameters = {'dig_time': 3.0}
        node = SimpleNamespace(
            results={}, parameter_names=('dig_time',),
            get_parameter=lambda name: SimpleNamespace(value=parameters[name]),
            make_routine=Mock(), publish=Mock(), state_pub=Mock(), mode=RobotState.DIG)
        goal = Mock()
        goal.goal_id.uuid = list(range(16))
        goal.request.index = 0
        RoutineServer.on_accepted(node, goal)
        parameters['dig_time'] = 10.0
        node.make_routine.assert_called_once_with(0, {'dig_time': 3.0})
        self.assertEqual(node.state_pub.publish.call_args.args[0].data, RobotState.DIG)
        goal.execute.assert_called_once()
        self.assertFalse(node.owns_mode)

    def test_finish_stops_and_returns_result_once(self):
        for outcome in ['succeeded', 'canceled', 'aborted']:
            with self.subTest(outcome=outcome):
                node = SimpleNamespace(
                    goal=Mock(), publish=Mock(), state_pub=Mock(),
                    owns_mode=True, mode=RobotState.DIG, state=RobotState.DIG,
                    action_type=SimpleNamespace(Result=lambda **kw: SimpleNamespace(**kw)),
                    result_future=Mock(), routine=Mock(), busy=True)
                goal = node.goal
                RoutineServer.finish(node, outcome)
                node.publish.assert_called_once_with(Command())
                node.state_pub.publish.assert_called_once()
                self.assertEqual(node.state_pub.publish.call_args.args[0].data,
                                 RobotState.TELEOP)
                result = node.result_future.set_result.call_args.args[0]
                self.assertEqual(result.success, outcome == 'succeeded')
                getattr(goal, {'succeeded': 'succeed', 'canceled': 'canceled',
                               'aborted': 'abort'}[outcome]).assert_called_once()
                RoutineServer.finish(node, outcome)
                node.result_future.set_result.assert_called_once()
                self.assertFalse(node.busy)

    def test_old_action_cannot_restore_over_new_state(self):
        node = SimpleNamespace(
            goal=Mock(), publish=Mock(), state_pub=Mock(), owns_mode=True,
            mode=RobotState.DIG, state=RobotState.DUMP,
            action_type=SimpleNamespace(Result=lambda **kw: SimpleNamespace(**kw)),
            result_future=Mock(), routine=Mock(), busy=True)
        RoutineServer.finish(node, 'aborted')
        node.state_pub.publish.assert_not_called()

    def test_commands_have_all_four_slots_and_zero_unused_axes(self):
        node = SimpleNamespace(velocity_pub=Mock(), arm_pub=Mock())
        RoutineServer.publish(node, Command(back_drum=-1.0))
        self.assertEqual(list(node.arm_pub.publish.call_args.args[0].data), [0, 0, 0, -1])
        velocity = node.velocity_pub.publish.call_args.args[0]
        self.assertEqual(velocity.linear.x, 0)
        self.assertEqual(velocity.angular.z, 0)


if __name__ == '__main__':
    unittest.main()
