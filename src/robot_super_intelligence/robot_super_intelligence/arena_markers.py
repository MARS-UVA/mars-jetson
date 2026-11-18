"""Resolve arena destinations from the same markers RViz displays."""

from copy import deepcopy
import math

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


def polygon_approach(marker, standoff=0.0):
    """Approach a polygon's highest-local-Y edge, facing inward.

    The goal is the edge midpoint, moved `standoff` meters outward along the
    edge normal. Visual line height is not goal height.
    """
    if not marker.header.frame_id or not math.isfinite(standoff):
        return None
    vertices = [(p.x, p.y) for p in marker.points]
    if vertices and vertices[0] == vertices[-1]:
        vertices.pop()
    p, q = marker.pose.position, marker.pose.orientation
    values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
    values.extend(v for point in vertices for v in point)
    if len(vertices) < 3 or not all(math.isfinite(v) for v in values):
        return None
    edges = [(a, b) for a, b in zip(vertices, vertices[1:] + vertices[:1]) if a != b]
    area = sum(a[0]*b[1] - b[0]*a[1] for a, b in edges)
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if abs(area) < 1e-9 or norm < 1e-9:
        return None
    a, b = max(edges, key=lambda edge: (edge[0][1] + edge[1][1]) / 2)
    mx, my = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
    nx, ny = b[1] - a[1], a[0] - b[0]
    cx = sum(x for x, _ in vertices) / len(vertices)
    cy = sum(y for _, y in vertices) / len(vertices)
    if nx*(cx-mx) + ny*(cy-my) < 0:
        nx, ny = -nx, -ny
    # (nx, ny) points inward; a standoff moves the goal the other way.
    length = math.hypot(nx, ny)
    mx -= nx / length * standoff
    my -= ny / length * standoff
    yaw = math.atan2(ny, nx)
    x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
    s, c = math.sin(yaw/2), math.cos(yaw/2)
    goal = PoseStamped()
    goal.header = deepcopy(marker.header)
    goal.pose.position.x = p.x + (1-2*(y*y+z*z))*mx + 2*(x*y-z*w)*my
    goal.pose.position.y = p.y + 2*(x*y+z*w)*mx + (1-2*(x*x+z*z))*my
    goal.pose.position.z = p.z + 2*(x*z-y*w)*mx + 2*(y*z+x*w)*my
    goal.pose.orientation.x = x*c + y*s
    goal.pose.orientation.y = y*c - x*s
    goal.pose.orientation.z = z*c + w*s
    goal.pose.orientation.w = w*c - z*s
    return goal


def construction_goal(marker):
    """Approach the construction zone polygon with no standoff."""
    return polygon_approach(marker)


def arrow_goal(marker):
    """Use an arrow marker's pose directly, with a normalized orientation."""
    if not marker.header.frame_id:
        return None
    p, q = marker.pose.position, marker.pose.orientation
    if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
        return None
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if norm < 1e-9:
        return None
    goal = PoseStamped(header=deepcopy(marker.header), pose=deepcopy(marker.pose))
    goal.pose.orientation.x /= norm
    goal.pose.orientation.y /= norm
    goal.pose.orientation.z /= norm
    goal.pose.orientation.w /= norm
    return goal


class ArenaMarkers:
    # key: (marker namespace, marker type) published by arena_markers_node.
    TRACKED = {
        'construction': ('Construction zone', Marker.LINE_STRIP),
        'berm': ('Berm', Marker.LINE_STRIP),
        'excavation': ('Excavation approach', Marker.ARROW),
        'start': ('Start approach', Marker.ARROW),
    }

    def __init__(self, node):
        self.node = node
        self.latest = {key: None for key in self.TRACKED}  # key -> (marker, received)
        self.subscription = node.create_subscription(
            MarkerArray, '/arena/markers', self.update, 1)

    def update(self, message):
        for marker in message.markers:
            for key, (namespace, kind) in self.TRACKED.items():
                current = self.latest[key]
                same = current is not None and (
                    (marker.ns, marker.id) == (current[0].ns, current[0].id))
                if marker.action == Marker.DELETEALL:
                    self.latest[key] = None
                elif marker.action == Marker.DELETE:
                    if same:
                        self.latest[key] = None
                elif marker.action == Marker.ADD:
                    if marker.ns == namespace and marker.type == kind:
                        self.latest[key] = (marker, self.node.get_clock().now())
                    elif same:
                        self.latest[key] = None

    def current(self, key):
        """Return the tracked marker unless it is missing or expired."""
        entry = self.latest[key]
        if entry is None:
            return None
        marker, received = entry
        lifetime = marker.lifetime.sec + marker.lifetime.nanosec / 1e9
        elapsed = (self.node.get_clock().now() - received).nanoseconds / 1e9
        if lifetime > 0 and elapsed >= lifetime:
            return None
        return marker

    def construction_goal(self):
        marker = self.current('construction')
        return None if marker is None else construction_goal(marker)

    def berm_goal(self, standoff=0.0):
        """Approach the berm's highest-local-Y edge, facing the berm.

        `standoff` (meters) backs the goal away from that edge so the
        robot's footprint can stay off the berm.
        """
        marker = self.current('berm')
        return None if marker is None else polygon_approach(marker, standoff)

    def excavation_goal(self):
        """Return the arena's explicit excavation approach while it is valid."""
        marker = self.current('excavation')
        return None if marker is None else arrow_goal(marker)

    def start_goal(self):
        """Return the arena's explicit start approach while it is valid."""
        marker = self.current('start')
        return None if marker is None else arrow_goal(marker)
