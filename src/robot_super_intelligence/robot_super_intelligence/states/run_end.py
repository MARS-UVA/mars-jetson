"""RunEnd state."""

from .base import State
from ..outcomes import Outcome


class RunEnd(State):
    outcomes = frozenset({Outcome.RUN_END, Outcome.FAILED, Outcome.NOT_IMPLEMENTED})

    def start(self, dispatch, on_complete):
        on_complete(Outcome.RUN_END)
