"""Navigate to the configured construction destination."""

from .traverse import Traverse
from ..outcomes import Outcome


class TraverseToConstruction(Traverse):
    success_outcome = Outcome.CONSTRUCTION_REACHED
    outcomes = frozenset({success_outcome, Outcome.FAILED})

    def __init__(self, construction_goal):
        super().__init__(construction_goal)
