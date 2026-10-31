"""TraverseToConstruction state."""

from .base import Placeholder
from ..outcomes import Outcome


class TraverseToConstruction(Placeholder):
    outcomes = frozenset({Outcome.CONSTRUCTION_REACHED, Outcome.FAILED, Outcome.NOT_IMPLEMENTED})
