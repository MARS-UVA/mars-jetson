"""Test-only machine: start approach, center berm, beaching check."""

from ..machine import Machine
from ..outcomes import Outcome
from ..states import IsBeached, RunEnd, TraverseToCenterBerm, TraverseToStart

# Meters from the berm's top edge to the robot's center. 0.9 keeps the
# 0.83 m half-length footprint (plus 0.03 m padding) off the berm.
BERM_STANDOFF = 0.9


class StartBermTestMachine(Machine):
    state_types = (TraverseToStart, TraverseToCenterBerm, IsBeached, RunEnd)
    initial_state = TraverseToStart
    end_state = RunEnd
    success_outcomes = frozenset({Outcome.NOT_BEACHED})
    transitions = {
        (TraverseToStart, Outcome.START_REACHED): TraverseToCenterBerm,
        (TraverseToCenterBerm, Outcome.BERM_REACHED): IsBeached,
        (IsBeached, Outcome.NOT_BEACHED): RunEnd,
        (TraverseToStart, Outcome.FAILED): RunEnd,
        (TraverseToCenterBerm, Outcome.FAILED): RunEnd,
    }

    @classmethod
    def valid(cls, request, arena_markers=None):
        return (arena_markers is not None
                and arena_markers.start_goal() is not None
                and arena_markers.berm_goal(BERM_STANDOFF) is not None)

    @classmethod
    def from_goal(cls, request, arena_markers=None):
        return cls(request.time_limit_sec, states={
            TraverseToStart: TraverseToStart(arena_markers.start_goal),
            TraverseToCenterBerm: TraverseToCenterBerm(
                lambda: arena_markers.berm_goal(BERM_STANDOFF)),
            IsBeached: IsBeached(),
            RunEnd: RunEnd(),
        })
