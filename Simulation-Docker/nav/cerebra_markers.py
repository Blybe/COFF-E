"""Publishes the Cerebra waypoints, paths and zones of the loaded map as RViz
markers on /waypoints (written by tools/cerebra_import.py to scene.json)."""

import json
import math
import sys

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

ZONE_COLORS = {
    "no_go_area": (0.9, 0.1, 0.1),
    "go_area": (0.1, 0.8, 0.2),
    "cover_area": (0.2, 0.4, 1.0),
    "map_layout": (0.6, 0.6, 0.6),
}


def marker(mid, ns, mtype, rgb, scale):
    m = Marker()
    m.header.frame_id = "map"
    m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
    m.color.r, m.color.g, m.color.b = rgb
    m.color.a = 1.0
    m.scale.x, m.scale.y, m.scale.z = scale
    m.pose.orientation.w = 1.0
    return m


def label(mid, text, x, y, rgb):
    m = marker(mid, "labels", Marker.TEXT_VIEW_FACING, rgb, (0.0, 0.0, 0.4))
    m.text = text
    m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.6
    return m


def build(scene):
    out, n = [], 0
    for wp in scene.get("waypoints", []):
        a = marker(n, "waypoints", Marker.ARROW, (0.1, 0.8, 0.2), (0.6, 0.12, 0.12))
        a.pose.position.x, a.pose.position.y, a.pose.position.z = wp["x"], wp["y"], 0.05
        a.pose.orientation.z, a.pose.orientation.w = math.sin(wp["yaw"] / 2), math.cos(wp["yaw"] / 2)
        out += [a, label(n + 1, wp["name"], wp["x"], wp["y"], (0.1, 0.8, 0.2))]
        n += 2
    for path in scene.get("paths", []):
        line = marker(n, "paths", Marker.LINE_STRIP, (1.0, 0.6, 0.0), (0.08, 0.0, 0.0))
        line.points = [Point(x=x, y=y, z=0.03) for x, y in path["points"]]
        x0, y0 = path["points"][0]
        out += [line, label(n + 1, f"path: {path['name']}", x0, y0, (1.0, 0.6, 0.0))]
        n += 2
    for zone in scene.get("zones", []):
        rgb = ZONE_COLORS.get(zone["type"], (0.8, 0.8, 0.8))
        line = marker(n, "zones", Marker.LINE_STRIP, rgb, (0.06, 0.0, 0.0))
        pts = zone["points"] + zone["points"][:1]
        line.points = [Point(x=x, y=y, z=0.03) for x, y in pts]
        cx = sum(p[0] for p in zone["points"]) / len(zone["points"])
        cy = sum(p[1] for p in zone["points"]) / len(zone["points"])
        out += [line, label(n + 1, f"{zone['name']} [{zone['type']}]", cx, cy, rgb)]
        n += 2
    return MarkerArray(markers=out)


class CerebraMarkers(Node):
    def __init__(self, scene_file):
        super().__init__("cerebra_markers")
        with open(scene_file, encoding="utf-8") as f:
            self.markers = build(json.load(f))
        self.pub = self.create_publisher(MarkerArray, "/waypoints", 1)
        # Re-published periodically so RViz windows opened later still get them
        self.create_timer(2.0, lambda: self.pub.publish(self.markers))


def main():
    rclpy.init()
    rclpy.spin(CerebraMarkers(sys.argv[1] if len(sys.argv) > 1 else "/tmp/cerebra/scene.json"))


if __name__ == "__main__":
    main()
