"""Short startup and construction-traversal sequence for focused testing."""

from ..machine import Machine
from ..outcomes import Outcome
from ..states import RunEnd, RunStart, TraverseToConstruction


class RunStartMachine(Machine):
    state_types = (RunStart, TraverseToConstruction, RunEnd)
    initial_state = RunStart
    end_state = RunEnd
    success_outcomes = frozenset({Outcome.CONSTRUCTION_REACHED})
    transitions = {
        (RunStart, Outcome.START_SUCCESS): TraverseToConstruction,
        (TraverseToConstruction, Outcome.CONSTRUCTION_REACHED): RunEnd,
        (RunStart, Outcome.FAILED): RunEnd,
        (TraverseToConstruction, Outcome.FAILED): RunEnd,
    }

    @classmethod
    def valid(cls, request, arena_markers=None):
        return arena_markers is not None and arena_markers.construction_goal() is not None

    @classmethod
    def from_goal(cls, request, arena_markers=None):
        return cls(request.time_limit_sec, states={
            RunStart: RunStart(),
            TraverseToConstruction: TraverseToConstruction(arena_markers.construction_goal),
            RunEnd: RunEnd(),
        })
