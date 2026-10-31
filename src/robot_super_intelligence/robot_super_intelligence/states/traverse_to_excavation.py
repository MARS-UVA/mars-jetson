"""TraverseToExcavation state."""

from .base import Placeholder
from ..outcomes import Outcome


class TraverseToExcavation(Placeholder):
    outcomes = frozenset({Outcome.EXCAVATION_REACHED, Outcome.FAILED, Outcome.NOT_IMPLEMENTED})
