"""States complete through futures rather than periodic orchestration ticks."""

from abc import ABC, abstractmethod

from rclpy.task import Future

from ..outcomes import Outcome


def completed(outcome):
    future = Future()
    future.set_result(outcome)
    return future


class State(ABC):
    outcomes = frozenset()

    def enter(self):
        """Reset per-invocation data; state objects are reused across cycles."""

    @abstractmethod
    def start(self, dispatch):
        """Return a future resolving to a declared Outcome; never block."""

    def cancel(self):
        """Release local work; the manager cancels the dispatched child action."""


class Placeholder(State):
    def start(self, dispatch):
        return completed(Outcome.NOT_IMPLEMENTED)
