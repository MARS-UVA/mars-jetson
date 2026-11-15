"""Base class for machine states that navigate to a supplied destination."""

from abc import abstractmethod

from autonomy_msgs.action import ManageRobot

from .state import State
from ..outcomes import Outcome


class Traverse(State):
    def __init__(self, goal_provider):
        if not callable(goal_provider):
            raise ValueError('A destination provider is required')
        self.goal_provider = goal_provider

    @property
    @abstractmethod
    def success_outcome(self):
        """Outcome reported when the destination is reached."""

    def start(self, dispatch, on_complete):
        target = self.goal_provider()
        if target is None:
            on_complete(Outcome.FAILED)
            return
        dispatch(ManageRobot.Goal(operation=ManageRobot.Goal.NAVIGATE,
                                  target_pose=target),
                 lambda outcome: on_complete(self.success_outcome
                     if outcome == 'SUCCESS' else Outcome.FAILED))
