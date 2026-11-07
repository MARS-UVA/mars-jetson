"""TraverseToConstruction state."""

from autonomy_msgs.action import ManageRobot

from .traverse import Traverse
from ..outcomes import Outcome


class TraverseToConstruction(Traverse):
    outcomes = frozenset({Outcome.CONSTRUCTION_REACHED, Outcome.FAILED, Outcome.NOT_IMPLEMENTED})

    def __init__(self, node=None, construction_goal=None):
        super().__init__(node)
        self.construction_goal = construction_goal

    def start(self, dispatch, on_complete):
        if self.construction_goal is None:
            return super().start(dispatch, on_complete)
        target = self.construction_goal()
        if target is None:
            on_complete(Outcome.FAILED)
            return
        dispatch(ManageRobot.Goal(operation=ManageRobot.Goal.NAVIGATE,
                                  target_pose=target),
                 lambda outcome: on_complete(Outcome.CONSTRUCTION_REACHED
                     if outcome == 'SUCCESS' else Outcome.FAILED))
