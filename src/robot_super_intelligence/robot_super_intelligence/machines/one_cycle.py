"""Dig, traverse, dump, and return to the excavation approach once."""

from .dig_traverse_dump import DigTraverseDumpMachine
from ..outcomes import Outcome
from ..states import DumpMaterial, Excavate, RunEnd, TraverseToConstruction, TraverseToExcavation


class OneCycleMachine(DigTraverseDumpMachine):
    state_types = (*DigTraverseDumpMachine.state_types, TraverseToExcavation)
    success_outcomes = frozenset({Outcome.EXCAVATION_REACHED})
    transitions = {
        **DigTraverseDumpMachine.transitions,
        (DumpMaterial, Outcome.DUMP_SUCCESS): TraverseToExcavation,
        (TraverseToExcavation, Outcome.EXCAVATION_REACHED): RunEnd,
        (TraverseToExcavation, Outcome.FAILED): RunEnd,
    }

    @classmethod
    def valid(cls, request, arena_markers=None):
        return (super().valid(request, arena_markers)
                and arena_markers.excavation_goal() is not None)

    @classmethod
    def from_goal(cls, request, arena_markers=None):
        return cls(request.time_limit_sec, states={
            Excavate: Excavate(routine_index=request.routine_index),
            TraverseToConstruction: TraverseToConstruction(arena_markers.construction_goal),
            DumpMaterial: DumpMaterial(routine_index=request.routine_index),
            TraverseToExcavation: TraverseToExcavation(arena_markers.excavation_goal),
            RunEnd: RunEnd(),
        })
