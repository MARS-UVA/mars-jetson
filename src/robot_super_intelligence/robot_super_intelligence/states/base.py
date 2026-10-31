"""Cooperative state lifecycle; each tick must return promptly."""

from abc import ABC, abstractmethod

from ..outcomes import Outcome


class State(ABC):
    """Return None while running, or one of outcomes when finished."""

    outcomes = frozenset()

    def enter(self):
        """Initialize this invocation of the state."""

    @abstractmethod
    def tick(self):
        """Advance without blocking ROS callbacks."""

    def cancel(self):
        """Cancel owned work; future child action handles belong here."""


class Placeholder(State):
    """Fail explicitly until an actual routine is implemented."""

    def tick(self):
        return Outcome.NOT_IMPLEMENTED
