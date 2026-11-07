"""One dig, construction traversal, and dump, then finish."""

from ..machine import Machine
from ..outcomes import Outcome
from ..states import DumpMaterial, Excavate, RunEnd, TraverseToConstruction


class DigTraverseDumpMachine(Machine):
    state_types = (Excavate, TraverseToConstruction, DumpMaterial, RunEnd)
    initial_state = Excavate
    end_state = RunEnd
    success_outcomes = frozenset({Outcome.DUMP_SUCCESS})
    transitions = {
        (Excavate, Outcome.EXCAVATE_SUCCESS): TraverseToConstruction,
        (TraverseToConstruction, Outcome.CONSTRUCTION_REACHED): DumpMaterial,
        (DumpMaterial, Outcome.DUMP_SUCCESS): RunEnd,
        (Excavate, Outcome.FAILED): RunEnd,
        (Excavate, Outcome.NOT_IMPLEMENTED): RunEnd,
        (TraverseToConstruction, Outcome.FAILED): RunEnd,
        (TraverseToConstruction, Outcome.NOT_IMPLEMENTED): RunEnd,
        (DumpMaterial, Outcome.FAILED): RunEnd,
        (DumpMaterial, Outcome.NOT_IMPLEMENTED): RunEnd,
    }

    @classmethod
    def valid(cls, request, arena_markers=None):
        return (Excavate().valid(request) and DumpMaterial().valid(request)
                and arena_markers is not None and arena_markers.construction_goal() is not None)

    @classmethod
    def from_goal(cls, request, arena_markers=None):
        return cls(request.time_limit_sec, states={
            Excavate: Excavate(routine_index=request.routine_index),
            TraverseToConstruction: TraverseToConstruction(
                construction_goal=arena_markers.construction_goal),
            DumpMaterial: DumpMaterial(routine_index=request.routine_index),
            RunEnd: RunEnd(),
        })

    def advance(self, outcome):
        previous = self.current
        super().advance(outcome)
        if previous is DumpMaterial and outcome is Outcome.DUMP_SUCCESS:
            self.completed_cycles += 1
