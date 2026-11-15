"""Dispatch a configured dump operation from a machine."""

from autonomy_msgs.action import ManageRobot

from .state import State
from ..outcomes import Outcome


class DumpMaterial(State):
    outcomes = frozenset({Outcome.DUMP_SUCCESS, Outcome.FAILED})

    def __init__(self, routine_index):
        if routine_index is None:
            raise ValueError('routine_index is required')
        self.routine_index = routine_index

    def start(self, dispatch, on_complete):
        dispatch(ManageRobot.Goal(operation=ManageRobot.Goal.DUMP,
                                  routine_index=self.routine_index),
                 lambda outcome: on_complete(Outcome.DUMP_SUCCESS
                     if outcome == 'SUCCESS' else Outcome.FAILED))
