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
        (RunStart, Outcome.NOT_IMPLEMENTED): RunEnd,
        (TraverseToConstruction, Outcome.FAILED): RunEnd,
        (TraverseToConstruction, Outcome.NOT_IMPLEMENTED): RunEnd,
    }
