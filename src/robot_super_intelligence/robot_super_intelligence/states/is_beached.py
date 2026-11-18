"""IsBeached state: transition check, currently always not beached."""

from .state import State
from ..outcomes import Outcome


class IsBeached(State):
    # Boilerplate until beaching detection exists: always report NOT_BEACHED.
    outcomes = frozenset({Outcome.NOT_BEACHED})

    def start(self, dispatch, on_complete):
        on_complete(Outcome.NOT_BEACHED)
