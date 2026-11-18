"""State configuration, navigation dispatch, and standalone pose validation."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from autonomy_msgs.action import ManageRobot
from robot_super_intelligence.states.state import State
from robot_super_intelligence.states import (
    Traverse, TraverseToConstruction, TraverseToExcavation, Excavate, DumpMaterial,
    TraverseToStart, TraverseToCenterBerm, IsBeached,
)
from robot_super_intelligence.machines import RunStartMachine
from robot_super_intelligence.outcomes import Outcome
from robot_super_intelligence.operations import NavigationOperation


@pytest.mark.parametrize('field,value', [
    ('frame', ''), ('position', float('nan')), ('orientation', 2.0), ('orientation', 0.0),
])
def test_traverse_rejects_invalid_poses(field, value):
    state = NavigationOperation(None)
    request = ManageRobot.Goal()
    request.target_pose.header.frame_id = 'map'
    request.target_pose.pose.orientation.w = 1.0
    assert state.valid(request)
    if field == 'frame':
        request.target_pose.header.frame_id = value
    elif field == 'position':
        request.target_pose.pose.position.x = value
    else:
        request.target_pose.pose.orientation.w = value
    assert not state.valid(request)


@pytest.mark.parametrize('state_type', [State, Traverse])
def test_incomplete_states_are_abstract(state_type):
    with pytest.raises(TypeError):
        state_type()


@pytest.mark.parametrize('state_type', [
    TraverseToConstruction, TraverseToExcavation, Excavate, DumpMaterial,
    TraverseToStart, TraverseToCenterBerm,
])
def test_states_require_configuration(state_type):
    with pytest.raises(TypeError):
        state_type()
    with pytest.raises(ValueError):
        state_type(None)


@pytest.mark.parametrize('state_type, expected', [
    (TraverseToConstruction, Outcome.CONSTRUCTION_REACHED),
    (TraverseToExcavation, Outcome.EXCAVATION_REACHED),
    (TraverseToStart, Outcome.START_REACHED),
    (TraverseToCenterBerm, Outcome.BERM_REACHED),
])
def test_configured_traversal_dispatches_and_maps_result(state_type, expected):
    target = ManageRobot.Goal().target_pose
    state = state_type(lambda: target)
    dispatch, complete = Mock(), Mock()
    state.start(dispatch, complete)
    request, callback = dispatch.call_args.args
    assert request.operation == ManageRobot.Goal.NAVIGATE
    assert request.target_pose == target
    complete.assert_not_called()
    callback('SUCCESS')
    complete.assert_called_once_with(expected)


@pytest.mark.parametrize('state_type', [
    TraverseToConstruction, TraverseToExcavation, TraverseToStart, TraverseToCenterBerm,
])
def test_traversal_navigation_failure_reports_failed(state_type):
    dispatch, complete = Mock(), Mock()
    state_type(lambda: ManageRobot.Goal().target_pose).start(dispatch, complete)
    _, callback = dispatch.call_args.args
    callback('CHILD_FAILED')
    complete.assert_called_once_with(Outcome.FAILED)
    assert Outcome.FAILED in state_type.outcomes


def test_is_beached_boilerplate_reports_not_beached_without_dispatch():
    dispatch, complete = Mock(), Mock()
    state = IsBeached()
    state.enter()
    state.start(dispatch, complete)
    dispatch.assert_not_called()
    complete.assert_called_once_with(Outcome.NOT_BEACHED)
    assert IsBeached.outcomes == {Outcome.NOT_BEACHED}


def test_destination_lost_after_acceptance_is_a_runtime_failure():
    dispatch, complete = Mock(), Mock()
    TraverseToConstruction(lambda: None).start(dispatch, complete)
    dispatch.assert_not_called()
    complete.assert_called_once_with(Outcome.FAILED)


def test_run_start_validates_and_constructs_configured_states():
    request = SimpleNamespace(time_limit_sec=10.0)
    target = ManageRobot.Goal().target_pose
    markers = SimpleNamespace(construction_goal=lambda: target)
    assert RunStartMachine.valid(request, markers)
    machine = RunStartMachine.from_goal(request, markers)
    traversal = machine.states[TraverseToConstruction]
    assert traversal.goal_provider() is target
    markers = SimpleNamespace(construction_goal=lambda: None)
    assert not RunStartMachine.valid(request, markers)
