"""Published polygon geometry determines the construction approach."""

import math
from types import SimpleNamespace

from geometry_msgs.msg import Point
import pytest
from rclpy.time import Time
from visualization_msgs.msg import Marker, MarkerArray

from robot_super_intelligence.arena_markers import (
    ArenaMarkers, construction_goal, polygon_approach,
)


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


def test_excavation_approach_validation_deletion_and_expiry():
    seconds = [0.0]
    clock = SimpleNamespace(now=lambda: Time(seconds=seconds[0]))
    node = SimpleNamespace(get_clock=lambda: clock,
                           create_subscription=lambda *args: None)
    source = ArenaMarkers(node)
    marker = Marker(ns='Excavation approach', id=0, type=Marker.ARROW)
    marker.header.frame_id = 'map'
    marker.pose.position.x, marker.pose.position.y = 2.5, 3.875
    marker.pose.orientation.z = 1.0
    marker.lifetime.sec = 1
    source.update(MarkerArray(markers=[marker]))
    goal = source.excavation_goal()
    assert goal.pose.position.x == 2.5
    assert goal.pose.position.y == 3.875
    assert goal.pose.orientation.z == 1.0
    seconds[0] = 2.0
    assert source.excavation_goal() is None
    source.update(MarkerArray(markers=[marker]))
    source.update(MarkerArray(markers=[Marker(ns=marker.ns, id=0, action=Marker.DELETE)]))
    assert source.excavation_goal() is None
    source.update(MarkerArray(markers=[marker]))
    source.update(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
    assert source.excavation_goal() is None
    marker.pose.position.x = float('nan')
    source.update(MarkerArray(markers=[marker]))
    assert source.excavation_goal() is None


def fake_node(seconds):
    clock = SimpleNamespace(now=lambda: Time(seconds=seconds[0]))
    return SimpleNamespace(get_clock=lambda: clock,
                           create_subscription=lambda *args: None)


def berm():
    # The NASA config's berm: 2.2 m x 0.9 m centered at (5.38, 0.75).
    marker = Marker(ns='Berm', id=6, type=Marker.LINE_STRIP)
    marker.header.frame_id = 'map'
    marker.pose.orientation.w = 1.0
    marker.points = [Point(x=x, y=y, z=0.05) for x, y in (
        (4.28, 0.3), (6.48, 0.3), (6.48, 1.2), (4.28, 1.2), (4.28, 0.3))]
    return marker


def test_standoff_moves_goal_outward_and_keeps_heading():
    goal = polygon_approach(polygon(), standoff=0.5)
    assert goal.pose.position.x == pytest.approx(12.0)
    assert goal.pose.position.y == pytest.approx(6.5)
    assert goal.pose.orientation.z == pytest.approx(-math.sqrt(0.5))
    assert polygon_approach(polygon(), standoff=float('nan')) is None


def test_berm_goal_centers_on_top_edge_facing_the_wall():
    source = ArenaMarkers(fake_node([0.0]))
    assert source.berm_goal() is None
    source.update(MarkerArray(markers=[berm()]))
    goal = source.berm_goal()
    assert goal.header.frame_id == 'map'
    assert goal.pose.position.x == pytest.approx(5.38)
    assert goal.pose.position.y == pytest.approx(1.2)
    assert goal.pose.orientation.z == pytest.approx(-math.sqrt(0.5))
    assert source.berm_goal(standoff=0.9).pose.position.y == pytest.approx(2.1)


def test_berm_and_construction_are_tracked_independently():
    source = ArenaMarkers(fake_node([0.0]))
    source.update(MarkerArray(markers=[polygon(), berm()]))
    assert source.construction_goal().pose.position.y == 6.0
    assert source.berm_goal().pose.position.y == pytest.approx(1.2)
    source.update(MarkerArray(markers=[Marker(ns='Berm', id=6, action=Marker.DELETE)]))
    assert source.berm_goal() is None
    assert source.construction_goal() is not None


def test_start_approach_validation_deletion_and_expiry():
    seconds = [0.0]
    source = ArenaMarkers(fake_node(seconds))
    assert source.start_goal() is None
    marker = Marker(ns='Start approach', id=0, type=Marker.ARROW)
    marker.header.frame_id = 'map'
    marker.pose.position.x, marker.pose.position.y = 1.5, 1.0
    marker.pose.orientation.z = 2.0  # Unnormalized on purpose.
    marker.lifetime.sec = 1
    source.update(MarkerArray(markers=[marker]))
    goal = source.start_goal()
    assert (goal.pose.position.x, goal.pose.position.y) == (1.5, 1.0)
    assert goal.pose.orientation.z == pytest.approx(1.0)
    assert source.excavation_goal() is None
    seconds[0] = 2.0
    assert source.start_goal() is None
    source.update(MarkerArray(markers=[marker]))
    source.update(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
    assert source.start_goal() is None
