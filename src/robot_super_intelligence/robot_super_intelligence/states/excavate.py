"""Excavate state."""

from .base import Placeholder
from ..outcomes import Outcome


class Excavate(Placeholder):
    outcomes = frozenset({Outcome.EXCAVATE_SUCCESS, Outcome.FAILED, Outcome.NOT_IMPLEMENTED})
