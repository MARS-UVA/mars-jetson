"""Complete excavation loop, ending when the run deadline is reached."""

from ..machine import Machine
from ..outcomes import Outcome
from ..states import (
    DumpMaterial, Excavate, RunEnd, RunStart,
    TraverseToConstruction, TraverseToExcavation,
)


class FullAutoMachine(Machine):
    state_types = (RunStart, TraverseToConstruction, DumpMaterial,
                   TraverseToExcavation, Excavate, RunEnd)
    initial_state = RunStart
    end_state = RunEnd
    success_outcomes = frozenset({Outcome.TIME_LIMIT_REACHED})
    transitions = {
        (RunStart, Outcome.START_SUCCESS): TraverseToConstruction,
        (TraverseToConstruction, Outcome.CONSTRUCTION_REACHED): DumpMaterial,
        (DumpMaterial, Outcome.DUMP_SUCCESS): TraverseToExcavation,
        (TraverseToExcavation, Outcome.EXCAVATION_REACHED): Excavate,
        (Excavate, Outcome.EXCAVATE_SUCCESS): TraverseToConstruction,
        (RunStart, Outcome.FAILED): RunEnd,
        (TraverseToConstruction, Outcome.FAILED): RunEnd,
        (DumpMaterial, Outcome.FAILED): RunEnd,
        (TraverseToExcavation, Outcome.FAILED): RunEnd,
        (Excavate, Outcome.FAILED): RunEnd,
    }

    def advance(self, outcome):
        previous = self.current
        super().advance(outcome)
        if previous is Excavate and outcome is Outcome.EXCAVATE_SUCCESS:
            self.completed_cycles += 1

    @classmethod
    def valid(cls, request, arena_markers=None):
        # No excavation destination is provided by the current goal interface.
        # Reject this unfinished machine before accepting or dispatching work.
        return False
