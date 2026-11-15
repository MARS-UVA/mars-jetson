"""RunStart state."""

from .state import State
from ..outcomes import Outcome


class RunStart(State):
    outcomes = frozenset({Outcome.START_SUCCESS})

    def start(self, dispatch, on_complete):
        on_complete(Outcome.START_SUCCESS)
