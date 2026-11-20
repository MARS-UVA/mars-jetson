"""LocalizeToAprilTags, its test machine, and the AprilTag helper."""

import math
from types import SimpleNamespace
from unittest.mock import Mock

from autonomy_msgs.action import ManageRobot
from geometry_msgs.msg import PoseStamped
import pytest

from robot_super_intelligence.machines import MACHINES, LocalizeTestMachine
from robot_super_intelligence.outcomes import Outcome
from robot_super_intelligence.states import LocalizeToAprilTags, RunEnd
from robot_super_intelligence.states.localize_to_apriltags import rotated


def yaw(pose):
    q = pose.pose.orientation
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def pose(x=1.0, y=2.0, heading=0.0):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(heading / 2), math.cos(heading / 2)
    return p


class FakeTags:
    """Answers gate calls from a script: True = a tag was seen at that heading."""

    def __init__(self, answers, pose_value=None):
        self.answers = list(answers)
        self.pose_value = pose() if pose_value is None else pose_value
        self.calls = []
        self.cancels = 0

    def call_gate(self, timeout_sec, callback):
        self.calls.append(timeout_sec)
        callback(self.answers.pop(0))

    def current_pose(self):
        return self.pose_value

    def cancel(self):
        self.cancels += 1

    def ready(self):
        return True


def run(state, nav_results=()):
    """Start the state; NAVIGATE dispatches answer from nav_results in order."""
    results = list(nav_results)
    goals, complete = [], Mock()

    def dispatch(goal, callback):
        goals.append(goal)
        callback(results.pop(0))

    state.enter()
    state.start(dispatch, complete)
    return goals, complete


def test_tag_in_view_localizes_without_turning():
    tags = FakeTags([True])
    goals, complete = run(LocalizeToAprilTags(tags))
    assert goals == []
    complete.assert_called_once_with(Outcome.APRILTAG_LOCALIZED)


def test_checks_ahead_then_45_one_way_then_45_the_other():
    tags = FakeTags([False, False, True])
    goals, complete = run(LocalizeToAprilTags(tags), ['SUCCESS', 'SUCCESS'])
    assert len(goals) == 2 and len(tags.calls) == 3
    for goal in goals:
        assert goal.operation == ManageRobot.Goal.NAVIGATE
        assert goal.target_pose.header.frame_id == 'map'
        assert (goal.target_pose.pose.position.x, goal.target_pose.pose.position.y) == (1.0, 2.0)
    assert [math.degrees(yaw(g.target_pose)) for g in goals] == pytest.approx([45, -45])
    complete.assert_called_once_with(Outcome.APRILTAG_LOCALIZED)


def test_gives_up_after_the_last_heading():
    tags = FakeTags([False] * 3)
    goals, complete = run(LocalizeToAprilTags(tags), ['SUCCESS'] * 2)
    assert len(goals) == 2 and len(tags.calls) == 3
    complete.assert_called_once_with(Outcome.APRILTAG_DETECTION_FAILED)


def test_headings_are_measured_from_where_the_search_started():
    # After the first turn the robot stops 10 deg short; the second target
    # must still be -45 deg from the start, not -45 deg from where it stopped.
    tags = FakeTags([False, False, False])
    poses = [pose(heading=math.radians(90)), pose(heading=math.radians(125))]
    tags.current_pose = lambda: poses.pop(0)
    goals, complete = run(LocalizeToAprilTags(tags), ['SUCCESS'] * 2)
    assert [math.degrees(yaw(g.target_pose)) for g in goals] == pytest.approx([135, 45])
    assert len(poses) == 1  # the pose was looked up once, at the first turn


def test_headings_can_be_configured():
    tags = FakeTags([False, False, False, True])
    headings = [math.radians(d) for d in (30, -30, 60)]
    goals, complete = run(LocalizeToAprilTags(tags, headings_rad=headings), ['SUCCESS'] * 3)
    assert [math.degrees(yaw(g.target_pose)) for g in goals] == pytest.approx([30, -30, 60])
    complete.assert_called_once_with(Outcome.APRILTAG_LOCALIZED)


def test_failed_turn_or_unknown_pose_fails():
    goals, complete = run(LocalizeToAprilTags(FakeTags([False])), ['CHILD_FAILED'])
    complete.assert_called_once_with(Outcome.APRILTAG_DETECTION_FAILED)
    tags = FakeTags([False])
    tags.pose_value = None
    tags.current_pose = lambda: None
    goals, complete = run(LocalizeToAprilTags(tags))
    assert goals == []
    complete.assert_called_once_with(Outcome.APRILTAG_DETECTION_FAILED)


def test_enter_resets_the_search_between_visits():
    tags = FakeTags([False, False, True, False, True])
    state = LocalizeToAprilTags(tags)
    run(state, ['SUCCESS', 'SUCCESS'])
    tags.pose_value = pose(heading=math.radians(-90))
    goals, complete = run(state, ['SUCCESS'])
    assert [math.degrees(yaw(g.target_pose)) for g in goals] == pytest.approx([-45])
    complete.assert_called_once_with(Outcome.APRILTAG_LOCALIZED)


def test_cancel_ignores_late_answers_and_cancels_the_gate_call():
    tags = FakeTags([])
    pending = []
    tags.call_gate = lambda timeout, callback: pending.append(callback)
    state = LocalizeToAprilTags(tags)
    complete = Mock()
    state.enter()
    state.start(Mock(), complete)
    state.cancel()
    pending[0](True)
    complete.assert_not_called()
    assert tags.cancels == 1


def test_rotated_keeps_position_and_wraps_heading():
    goal = rotated(pose(3.0, 4.0, math.radians(170)), math.radians(36))
    assert (goal.pose.position.x, goal.pose.position.y) == (3.0, 4.0)
    assert yaw(goal) == pytest.approx(math.radians(-154))


def test_settings_are_validated():
    with pytest.raises(ValueError):
        LocalizeToAprilTags(None)
    with pytest.raises(ValueError):
        LocalizeToAprilTags(FakeTags([]), gate_timeout_sec=0)
    with pytest.raises(ValueError):
        LocalizeToAprilTags(FakeTags([]), headings_rad=[float('nan')])
    assert LocalizeToAprilTags.outcomes == {
        Outcome.APRILTAG_LOCALIZED, Outcome.APRILTAG_DETECTION_FAILED}


def test_localize_test_machine():
    request = SimpleNamespace(time_limit_sec=60.0)
    assert MACHINES['localize_test'] is LocalizeTestMachine
    assert LocalizeTestMachine.valid(request, None, apriltags=FakeTags([]))
    assert not LocalizeTestMachine.valid(request, None)
    for outcome in (Outcome.APRILTAG_LOCALIZED, Outcome.APRILTAG_DETECTION_FAILED):
        machine = LocalizeTestMachine.from_goal(request, apriltags=FakeTags([]))
        machine.advance(outcome)
        assert machine.current is RunEnd
        machine.advance(Outcome.RUN_END)
        assert machine.done and machine.result is outcome
    assert LocalizeTestMachine.success_outcomes == {Outcome.APRILTAG_LOCALIZED}
