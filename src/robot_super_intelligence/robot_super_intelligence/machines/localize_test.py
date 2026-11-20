"""Test-only machine: localize to AprilTags from wherever the robot is."""

from ..machine import Machine
from ..outcomes import Outcome
from ..states import LocalizeToAprilTags, RunEnd


class LocalizeTestMachine(Machine):
    state_types = (LocalizeToAprilTags, RunEnd)
    initial_state = LocalizeToAprilTags
    end_state = RunEnd
    success_outcomes = frozenset({Outcome.APRILTAG_LOCALIZED})
    transitions = {
        (LocalizeToAprilTags, Outcome.APRILTAG_LOCALIZED): RunEnd,
        (LocalizeToAprilTags, Outcome.APRILTAG_DETECTION_FAILED): RunEnd,
    }

    @classmethod
    def valid(cls, request, arena_markers=None, apriltags=None):
        return apriltags is not None and apriltags.ready()

    @classmethod
    def from_goal(cls, request, arena_markers=None, apriltags=None):
        return cls(request.time_limit_sec, states={
            LocalizeToAprilTags: LocalizeToAprilTags(apriltags),
            RunEnd: RunEnd(),
        })
