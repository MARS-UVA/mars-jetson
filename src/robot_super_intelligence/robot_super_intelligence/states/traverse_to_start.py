"""Navigate to the start approach, where the start-zone AprilTag is visible."""

from .traverse import Traverse
from ..outcomes import Outcome


class TraverseToStart(Traverse):
    success_outcome = Outcome.START_REACHED
    outcomes = frozenset({success_outcome, Outcome.FAILED})

    def __init__(self, start_goal):
        super().__init__(start_goal)
