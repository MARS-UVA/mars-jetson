"""Dispatch a configured dig operation from a machine."""

from autonomy_msgs.action import ManageRobot

from .state import State
from ..outcomes import Outcome


class Excavate(State):
    outcomes = frozenset({Outcome.EXCAVATE_SUCCESS, Outcome.FAILED})

    def __init__(self, routine_index):
        if routine_index is None:
            raise ValueError('routine_index is required')
        self.routine_index = routine_index

    def start(self, dispatch, on_complete):
        dispatch(ManageRobot.Goal(operation=ManageRobot.Goal.DIG,
                                  routine_index=self.routine_index),
                 lambda outcome: on_complete(Outcome.EXCAVATE_SUCCESS
                     if outcome == 'SUCCESS' else Outcome.FAILED))
