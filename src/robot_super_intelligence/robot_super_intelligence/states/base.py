"""States report completion through callbacks rather than polling."""

from abc import ABC, abstractmethod

from ..outcomes import Outcome


class State(ABC):
    outcomes = frozenset()

    def enter(self):
        """Reset per-invocation data; state objects are reused across cycles."""

    @abstractmethod
    def start(self, dispatch, on_complete):
        """Call on_complete with a declared Outcome when finished; never block."""

    def cancel(self):
        """Release local work; the manager cancels the dispatched child action."""


class Placeholder(State):
    def start(self, dispatch, on_complete):
        on_complete(Outcome.NOT_IMPLEMENTED)
