"""TraverseToConstruction state."""

from .traverse import Traverse
from ..outcomes import Outcome


class TraverseToConstruction(Traverse):
    outcomes = frozenset({Outcome.CONSTRUCTION_REACHED, Outcome.FAILED, Outcome.NOT_IMPLEMENTED})
