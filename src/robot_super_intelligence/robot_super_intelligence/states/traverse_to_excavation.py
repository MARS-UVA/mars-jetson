"""TraverseToExcavation state."""

from .traverse import Traverse
from ..outcomes import Outcome


class TraverseToExcavation(Traverse):
    outcomes = frozenset({Outcome.EXCAVATION_REACHED, Outcome.FAILED, Outcome.NOT_IMPLEMENTED})
