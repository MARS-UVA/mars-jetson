"""Transition graph advanced by state completion events."""

import math
import time

from .outcomes import Outcome
from .states import (
    DumpMaterial, Excavate, RunEnd, RunStart,
    TraverseToConstruction, TraverseToExcavation,
)

STATE_TYPES = (RunStart, TraverseToConstruction, DumpMaterial,
               TraverseToExcavation, Excavate, RunEnd)
TRANSITIONS = {
    (RunStart, Outcome.START_SUCCESS): TraverseToConstruction,
    (TraverseToConstruction, Outcome.CONSTRUCTION_REACHED): DumpMaterial,
    (DumpMaterial, Outcome.DUMP_SUCCESS): TraverseToExcavation,
    (TraverseToExcavation, Outcome.EXCAVATION_REACHED): Excavate,
    (Excavate, Outcome.EXCAVATE_SUCCESS): TraverseToConstruction,
}
for state_type in STATE_TYPES[:-1]:
    for outcome in (Outcome.FAILED, Outcome.NOT_IMPLEMENTED):
        TRANSITIONS[state_type, outcome] = RunEnd


class Machine:
    """Consume state outcomes; the manager owns the run deadline."""

    def __init__(self, time_limit_sec, states=None, clock=time.monotonic):
        if not math.isfinite(time_limit_sec) or time_limit_sec <= 0:
            raise ValueError('time_limit_sec must be positive and finite')
        self.clock = clock
        self.started = clock()
        self.time_limit_sec = time_limit_sec
        self.states = states if states is not None else {cls: cls() for cls in STATE_TYPES}
        self.current = RunStart
        self.latest_outcome = ''
        self.completed_cycles = 0
        self.result = None
        self.done = False
        self.state.enter()

    @property
    def state(self):
        return self.states[self.current]

    @property
    def elapsed(self):
        return max(0.0, self.clock() - self.started)

    def end(self, outcome):
        if self.done or self.current is RunEnd:
            return
        try:
            self.state.cancel()
        finally:
            self.result = outcome
            self.latest_outcome = outcome.value
            self.current = RunEnd
            self.state.enter()

    def advance(self, outcome):
        """Consume one completed state's outcome."""
        if self.done:
            return
        if outcome not in self.state.outcomes:
            raise ValueError(f'{self.current.__name__} returned undeclared outcome: {outcome}')
        self.latest_outcome = outcome.value
        if self.current is RunEnd:
            if outcome is not Outcome.RUN_END:
                self.result = Outcome.FAILED
            self.done = True
            return
        target = TRANSITIONS[self.current, outcome]
        if target is RunEnd:
            self.end(outcome)
            return
        if self.current is Excavate:
            self.completed_cycles += 1
        self.current = target
        self.state.enter()
