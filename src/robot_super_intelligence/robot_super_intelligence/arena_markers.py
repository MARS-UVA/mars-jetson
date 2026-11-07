"""Resolve the construction approach from the same markers RViz displays."""

from copy import deepcopy
import math

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


def construction_goal(marker):
    """Use the highest-local-Y edge; visual line height is not goal height."""
    if not marker.header.frame_id:
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


class ArenaMarkers:
    def __init__(self, node):
        self.node = node
        self.marker = None
        self.received = None
        self.subscription = node.create_subscription(
            MarkerArray, '/arena/markers', self.update, 1)

    def update(self, message):
        for marker in message.markers:
            current = self.marker
            same = current is not None and (marker.ns, marker.id) == (current.ns, current.id)
            if marker.action == Marker.DELETEALL:
                self.marker = None
            elif marker.action == Marker.DELETE:
                if same:
                    self.marker = None
            elif marker.action == Marker.ADD:
                if marker.ns == 'Construction zone' and marker.type == Marker.LINE_STRIP:
                    self.marker = marker
                    self.received = self.node.get_clock().now()
                elif same:
                    self.marker = None

    def construction_goal(self):
        if self.marker is None:
            return None
        lifetime = self.marker.lifetime.sec + self.marker.lifetime.nanosec / 1e9
        elapsed = (self.node.get_clock().now() - self.received).nanoseconds / 1e9
        if lifetime > 0 and elapsed >= lifetime:
            return None
        return construction_goal(self.marker)
