"""Navigate to the configured excavation destination."""

from .traverse import Traverse
from ..outcomes import Outcome


class TraverseToExcavation(Traverse):
    success_outcome = Outcome.EXCAVATION_REACHED
    outcomes = frozenset({success_outcome, Outcome.FAILED})

    def __init__(self, excavation_goal):
        super().__init__(excavation_goal)
