"""Navigation state validates poses before dispatching them to Nav2."""

from autonomy_msgs.action import ManageRobot
import pytest

from robot_super_intelligence.states import Traverse


@pytest.mark.parametrize('field,value', [
    ('frame', ''), ('position', float('nan')), ('orientation', 2.0), ('orientation', 0.0),
])
def test_traverse_rejects_invalid_poses(field, value):
    state = Traverse()
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
