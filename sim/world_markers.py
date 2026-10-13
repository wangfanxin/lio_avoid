#!/usr/bin/env python3
"""Publish the world from `worlds/pillars.yaml` as RViz markers, in W.

RViz cannot ray-cast, but it can draw: this node is what turns the file into something to look at
next to the simulated scan, so "does the cloud match the world" is a question the eye can answer in a
second -- and the pillar-fit tool's numbers are the same question asked precisely.

usage: sim/world_markers.py [--world ...] [--square-from-mission]
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from sim import raycast          # noqa: E402

try:
    import yaml
except ImportError:              # pragma: no cover
    yaml = None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                    "..", "worlds", "pillars.yaml"))
    ap.add_argument("--frame", default="world_w")
    ap.add_argument("--topic", default="/sim/world/markers")
    ap.add_argument("--latch-seconds", type=float, default=0.0,
                    help="keep republishing for this long (0 = forever at 1 Hz)")
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from visualization_msgs.msg import Marker, MarkerArray
    from geometry_msgs.msg import Point

    world = raycast.World.load(args.world)
    cfg = {}
    if yaml is not None:
        try:
            cfg = yaml.safe_load(open(args.world)) or {}
        except OSError:
            cfg = {}

    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    except Exception:
        rclpy.init(args=[])
    node = Node("world_markers")
    qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(MarkerArray, args.topic, qos)

    msgs = MarkerArray()
    mid = 0

    # pillars as cylinders, the tallest first so RViz's alpha blending looks sane
    for cx, cy, r, z0, z1 in world.pillars:
        m = Marker()
        m.header.frame_id = args.frame
        m.ns = "pillars"
        m.id = mid
        mid += 1
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = float(cx)
        m.pose.position.y = float(cy)
        m.pose.position.z = float((z0 + z1) / 2.0)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = float(2 * r)
        m.scale.z = float(z1 - z0)
        m.color.r, m.color.g, m.color.b, m.color.a = 0.55, 0.55, 0.6, 0.85
        msgs.markers.append(m)

    # the arena: floor and ceiling as thin boxes, so the vehicle has a visible room
    for z_plane, colour in ((world.floor_z, (0.25, 0.35, 0.25)), (world.ceiling_z, (0.3, 0.3, 0.4))):
        if z_plane is None:
            continue
        m = Marker()
        m.header.frame_id = args.frame
        m.ns = "arena"
        m.id = mid
        mid += 1
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = float((world.walls["x"][0] + world.walls["x"][1]) / 2) if world.walls else 0.0
        m.pose.position.y = float((world.walls["y"][0] + world.walls["y"][1]) / 2) if world.walls else 0.0
        m.pose.position.z = float(z_plane) - (0.02 if z_plane == world.floor_z else -0.02)
        m.pose.orientation.w = 1.0
        m.scale.x = float(world.walls["x"][1] - world.walls["x"][0]) if world.walls else 10.0
        m.scale.y = float(world.walls["y"][1] - world.walls["y"][0]) if world.walls else 10.0
        m.scale.z = 0.02
        m.color.r, m.color.g, m.color.b, m.color.a = colour[0], colour[1], colour[2], 0.35
        msgs.markers.append(m)

    # the mission square, so the trajectory can be seen against the forest
    mission = (cfg.get("mission") or {}) if isinstance(cfg, dict) else {}
    square = mission.get("square") or (cfg.get("square") if isinstance(cfg, dict) else None)
    if square:
        sq = np.asarray(square, dtype=float).reshape(-1, 2)
        sq = np.vstack([sq, sq[:1]])
        m = Marker()
        m.header.frame_id = args.frame
        m.ns = "mission"
        m.id = mid
        mid += 1
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.02
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.85, 0.1, 1.0
        m.pose.orientation.w = 1.0
        for x, y in sq:
            m.points.append(Point(x=float(x), y=float(y), z=0.6))
        msgs.markers.append(m)

    print("world_markers: %d pillars, %d markers, frame %s, topic %s"
          % (len(world.pillars), len(msgs.markers), args.frame, args.topic), flush=True)
    print("  arena %s, floor %.2f, ceiling %s, square %s"
          % (world.walls, world.floor_z, world.ceiling_z, "yes" if square else "no (add mission.square)"),
          flush=True)

    import time
    t0 = time.monotonic()
    period = 1.0
    while args.latch_seconds == 0.0 or (time.monotonic() - t0) < args.latch_seconds:
        stamp = node.get_clock().now().to_msg()
        for m in msgs.markers:
            m.header.stamp = stamp
        pub.publish(msgs)
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(period)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
