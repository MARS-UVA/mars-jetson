"""Graph and configuration of the start/berm test machine."""

from types import SimpleNamespace

from autonomy_msgs.action import ManageRobot

from robot_super_intelligence.machines import MACHINES, StartBermTestMachine
from robot_super_intelligence.machines.start_berm_test import BERM_STANDOFF
from robot_super_intelligence.outcomes import Outcome
from robot_super_intelligence.states import (
    IsBeached, RunEnd, TraverseToCenterBerm, TraverseToStart,
)


def markers(start=True, berm=True):
    pose = ManageRobot.Goal().target_pose
    calls = []

    def berm_goal(standoff=0.0):
        calls.append(standoff)
        return pose if berm else None

    return SimpleNamespace(start_goal=lambda: pose if start else None,
                           berm_goal=berm_goal, calls=calls, pose=pose)


def request():
    return SimpleNamespace(time_limit_sec=60.0)


def test_registered_and_validated():
    assert MACHINES['start_berm_test'] is StartBermTestMachine
    assert StartBermTestMachine.valid(request(), markers())
    assert not StartBermTestMachine.valid(request(), None)
    assert not StartBermTestMachine.valid(request(), markers(start=False))
    assert not StartBermTestMachine.valid(request(), markers(berm=False))


def test_berm_traversal_uses_the_standoff():
    source = markers()
    machine = StartBermTestMachine.from_goal(request(), source)
    assert machine.states[TraverseToCenterBerm].goal_provider() is source.pose
    assert source.calls[-1] == BERM_STANDOFF


def test_success_path_ends_with_not_beached():
    machine = StartBermTestMachine.from_goal(request(), markers())
    for state, outcome in ((TraverseToStart, Outcome.START_REACHED),
                           (TraverseToCenterBerm, Outcome.BERM_REACHED),
                           (IsBeached, Outcome.NOT_BEACHED)):
        assert machine.current is state
        machine.advance(outcome)
    assert machine.current is RunEnd
    machine.advance(Outcome.RUN_END)
    assert machine.done and machine.result is Outcome.NOT_BEACHED


def test_navigation_failure_ends_the_run_as_failed():
    machine = StartBermTestMachine.from_goal(request(), markers())
    machine.advance(Outcome.START_REACHED)
    machine.advance(Outcome.FAILED)
    assert machine.current is RunEnd
    machine.advance(Outcome.RUN_END)
    assert machine.done and machine.result is Outcome.FAILED
