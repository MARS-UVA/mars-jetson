"""RunStart state."""

from .base import State, completed
from ..outcomes import Outcome


class RunStart(State):
    outcomes = frozenset({Outcome.START_SUCCESS, Outcome.FAILED, Outcome.NOT_IMPLEMENTED})

    def start(self, dispatch):
        return completed(Outcome.START_SUCCESS)
