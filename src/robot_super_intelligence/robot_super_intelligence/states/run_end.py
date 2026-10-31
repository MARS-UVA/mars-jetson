"""RunEnd state."""

from .base import State
from ..outcomes import Outcome


class RunEnd(State):
    outcomes = frozenset({Outcome.RUN_END, Outcome.FAILED, Outcome.NOT_IMPLEMENTED})

    def tick(self):
        return Outcome.RUN_END
