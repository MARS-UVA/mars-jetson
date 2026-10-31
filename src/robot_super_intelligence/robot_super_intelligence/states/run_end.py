"""RunEnd state."""

from .base import State, completed
from ..outcomes import Outcome


class RunEnd(State):
    outcomes = frozenset({Outcome.RUN_END, Outcome.FAILED, Outcome.NOT_IMPLEMENTED})

    def start(self, dispatch):
        return completed(Outcome.RUN_END)
