"""RunEnd state."""

from .state import State
from ..outcomes import Outcome


class RunEnd(State):
    outcomes = frozenset({Outcome.RUN_END})

    def start(self, dispatch, on_complete):
        on_complete(Outcome.RUN_END)
