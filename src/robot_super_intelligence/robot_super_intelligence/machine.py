"""Transition graph advanced by state completion events."""

import math
import time

from .outcomes import Outcome


class Machine:
    """Execute the states and transitions supplied by a concrete machine."""

    success_outcomes = frozenset()

    def __init__(self, time_limit_sec, states=None, clock=time.monotonic):
        if not math.isfinite(time_limit_sec) or time_limit_sec <= 0:
            raise ValueError('time_limit_sec must be positive and finite')
        self.clock = clock
        self.started = clock()
        self.time_limit_sec = time_limit_sec
        self.states = states if states is not None else {cls: cls() for cls in self.state_types}
        self.current = self.initial_state
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
        if self.done or self.current is self.end_state:
            return
        try:
            self.state.cancel()
        finally:
            self.result = outcome
            self.latest_outcome = outcome.value
            self.current = self.end_state
            self.state.enter()

    def advance(self, outcome):
        """Consume one completed state's outcome."""
        if self.done:
            return
        if outcome not in self.state.outcomes:
            raise ValueError(f'{self.current.__name__} returned undeclared outcome: {outcome}')
        self.latest_outcome = outcome.value
        if self.current is self.end_state:
            if outcome is not Outcome.RUN_END:
                self.result = Outcome.FAILED
            elif self.result is None:
                self.result = outcome
            self.done = True
            return
        target = self.transitions[self.current, outcome]
        if target is self.end_state:
            if outcome not in self.success_outcomes:
                self.end(outcome)
                return
            self.result = outcome
        self.current = target
        self.state.enter()
