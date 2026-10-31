"""RunStart state."""

from .base import State
from ..outcomes import Outcome


class RunStart(State):
    outcomes = frozenset({Outcome.START_SUCCESS, Outcome.FAILED, Outcome.NOT_IMPLEMENTED})

    def tick(self):
        return Outcome.START_SUCCESS
