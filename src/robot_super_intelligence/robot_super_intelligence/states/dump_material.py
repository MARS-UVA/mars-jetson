"""DumpMaterial state."""

from .base import Placeholder
from ..outcomes import Outcome


class DumpMaterial(Placeholder):
    outcomes = frozenset({Outcome.DUMP_SUCCESS, Outcome.FAILED, Outcome.NOT_IMPLEMENTED})
