"""One cycle includes the return traversal and requires its arena destination."""

from types import SimpleNamespace

from autonomy_msgs.action import ManageRobot
from geometry_msgs.msg import PoseStamped
from robot_super_intelligence.machines import MACHINES
from robot_super_intelligence.outcomes import Outcome
from robot_super_intelligence.states import (
    DumpMaterial, Excavate, RunEnd, TraverseToConstruction, TraverseToExcavation,
)


def arena(goal):
    return SimpleNamespace(construction_goal=lambda: PoseStamped(),
                           excavation_goal=lambda: goal)


def test_one_cycle_requires_excavation_approach():
    machine = MACHINES['one_cycle']
    request = ManageRobot.Goal(routine_index=0, time_limit_sec=120.0)
    assert not machine.valid(request, arena(None))
    assert machine.valid(request, arena(PoseStamped()))


def test_one_cycle_returns_after_dump_and_finishes_only_after_arrival():
    cls = MACHINES['one_cycle']
    machine = cls.from_goal(ManageRobot.Goal(time_limit_sec=120.0), arena(PoseStamped()))
    for state, outcome in (
        (Excavate, Outcome.EXCAVATE_SUCCESS),
        (TraverseToConstruction, Outcome.CONSTRUCTION_REACHED),
        (DumpMaterial, Outcome.DUMP_SUCCESS),
    ):
        assert machine.current is state
        machine.advance(outcome)
    assert machine.current is TraverseToExcavation
    assert machine.result is None
    assert machine.completed_cycles == 1
    machine.advance(Outcome.EXCAVATION_REACHED)
    assert machine.current is RunEnd
    machine.advance(Outcome.RUN_END)
    assert machine.done
    assert machine.result is Outcome.EXCAVATION_REACHED


def test_one_cycle_has_transitions_for_all_declared_outcomes():
    cls = MACHINES['one_cycle']
    for state in cls.state_types:
        if state is not cls.end_state:
            for outcome in state.outcomes:
                assert (state, outcome) in cls.transitions
