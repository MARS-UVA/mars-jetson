"""Navigate to the center of the berm's approach edge, facing the berm."""

from .traverse import Traverse
from ..outcomes import Outcome


class TraverseToCenterBerm(Traverse):
    success_outcome = Outcome.BERM_REACHED
    outcomes = frozenset({success_outcome, Outcome.FAILED})

    def __init__(self, berm_goal):
        super().__init__(berm_goal)
