#!/usr/bin/env python3
"""Publish fixed arena outlines and labels for RViz."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from arena_configs import ARENA_CONFIGS


class ArenaMarkersNode(Node):
    def __init__(self):
        super().__init__('arena_markers_node')
        arena_name = self.declare_parameter('arena_name', 'ARENA_NASA').value
        if arena_name not in ARENA_CONFIGS:
            raise ValueError(f'Unknown arena_name {arena_name!r}; choose from {list(ARENA_CONFIGS)}')
        config = ARENA_CONFIGS[arena_name]
        regions = dict(config['zones'])
        cx, cy = config['berm']['center']
        width, height = config['berm']['dimensions']
        regions['Berm'] = [
            (cx - width / 2, cy - height / 2),
            (cx + width / 2, cy - height / 2),
            (cx + width / 2, cy + height / 2),
            (cx - width / 2, cy + height / 2),
        ]
        colors = [(0.2, 0.8, 0.2), (1.0, 0.55, 0.0), (0.2, 0.6, 1.0), (0.9, 0.2, 0.8)]
        self.markers = MarkerArray()
        for index, (name, vertices) in enumerate(regions.items()):
            # Vertex averages keep labels inside these simple arena regions.
            label_x = sum(x for x, _ in vertices) / len(vertices)
            label_y = sum(y for _, y in vertices) / len(vertices)
            for offset, kind in enumerate((Marker.LINE_STRIP, Marker.TEXT_VIEW_FACING)):
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.ns = name
                marker.id = index * 2 + offset
                marker.type = kind
                marker.action = Marker.ADD
                marker.pose.orientation.w = 1.0
                marker.color.r, marker.color.g, marker.color.b = colors[index]
                marker.color.a = 1.0
                # Default zero lifetime keeps markers visible between publications.
                if kind == Marker.LINE_STRIP:
                    marker.scale.x = 0.03
                    marker.points = [Point(x=float(x), y=float(y), z=0.05)
                                     for x, y in [*vertices, vertices[0]]]
                else:
                    marker.text = name
                    marker.scale.z = 0.2
                    marker.pose.position = Point(x=label_x, y=label_y, z=0.15)
                self.markers.markers.append(marker)
        self.publisher = self.create_publisher(MarkerArray, '/arena/markers', 1)
        self.publish_markers()
        self.timer = self.create_timer(1.0, self.publish_markers)

    def publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        for marker in self.markers.markers:
            marker.header.stamp = stamp
        self.publisher.publish(self.markers)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ArenaMarkersNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
