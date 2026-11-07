"""Published polygon geometry determines the construction approach."""

import math
from types import SimpleNamespace

from geometry_msgs.msg import Point
import pytest
from rclpy.time import Time
from visualization_msgs.msg import Marker, MarkerArray

from robot_super_intelligence.arena_markers import ArenaMarkers, construction_goal


def polygon():
    marker = Marker(ns='Construction zone', id=42, type=Marker.LINE_STRIP)
    marker.header.frame_id = 'arena'
    marker.pose.orientation.w = 1.0
    marker.points = [Point(x=x, y=y, z=0.05) for x, y in (
        (10., 4.), (14., 4.), (14., 6.), (10., 6.), (10., 4.))]
    return marker


def test_goal_comes_from_polygon_and_ignores_visual_height():
    goal = construction_goal(polygon())
    assert goal.header.frame_id == 'arena'
    assert goal.pose.position.x == 12.0
    assert goal.pose.position.y == 6.0
    assert goal.pose.position.z == 0.0
    assert goal.pose.orientation.z == pytest.approx(-math.sqrt(0.5))
    assert goal.pose.orientation.w == pytest.approx(math.sqrt(0.5))


def test_marker_pose_rotates_and_translates_goal_and_heading():
    marker = polygon()
    marker.pose.position.x = 3.0
    marker.pose.position.y = 2.0
    marker.pose.orientation.z = math.sqrt(0.5)
    marker.pose.orientation.w = math.sqrt(0.5)
    goal = construction_goal(marker)
    assert goal.pose.position.x == pytest.approx(-3.0)
    assert goal.pose.position.y == pytest.approx(14.0)
    assert goal.pose.orientation.z == pytest.approx(0.0, abs=1e-9)
    assert goal.pose.orientation.w == pytest.approx(1.0)


def test_reversed_polygon_still_faces_inward():
    marker = polygon()
    marker.points.reverse()
    goal = construction_goal(marker)
    assert goal.pose.position.x == 12.0
    assert goal.pose.position.y == 6.0
    assert goal.pose.orientation.z == pytest.approx(-math.sqrt(0.5))


@pytest.mark.parametrize('invalid', ['frame', 'points', 'nan', 'quaternion', 'degenerate'])
def test_invalid_geometry_has_no_goal(invalid):
    marker = polygon()
    if invalid == 'frame':
        marker.header.frame_id = ''
    elif invalid == 'points':
        marker.points = []
    elif invalid == 'nan':
        marker.points[0].x = float('nan')
    elif invalid == 'quaternion':
        marker.pose.orientation.w = 0.0
    elif invalid == 'degenerate':
        for p in marker.points:
            p.y = 0.0
    assert construction_goal(marker) is None


def test_subscriber_updates_and_honors_deletion_and_expiry():
    seconds = [0.0]
    clock = SimpleNamespace(now=lambda: Time(seconds=seconds[0]))
    node = SimpleNamespace(get_clock=lambda: clock,
                           create_subscription=lambda *args: None)
    source = ArenaMarkers(node)
    assert source.construction_goal() is None
    marker = polygon()
    source.update(MarkerArray(markers=[marker]))
    assert source.construction_goal().pose.position.x == 12.0
    marker.pose.position.x = 4.0
    marker.lifetime.sec = 1
    source.update(MarkerArray(markers=[marker]))
    assert source.construction_goal().pose.position.x == 16.0
    seconds[0] = 2.0
    assert source.construction_goal() is None
    source.update(MarkerArray(markers=[marker]))
    assert source.construction_goal() is not None
    source.update(MarkerArray(markers=[Marker(ns=marker.ns, id=marker.id, action=Marker.DELETE)]))
    assert source.construction_goal() is None
    source.update(MarkerArray(markers=[marker]))
    source.update(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
    assert source.construction_goal() is None


def test_label_only_does_not_supply_region_geometry():
    node = SimpleNamespace(create_subscription=lambda *args: None)
    source = ArenaMarkers(node)
    source.update(MarkerArray(markers=[Marker(ns='Construction zone', type=Marker.TEXT_VIEW_FACING)]))
    assert source.construction_goal() is None
