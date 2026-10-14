#!/usr/bin/env python3
"""Republish the LIO's registered scan under the name the planner subscribes to.

WHY THIS EXISTS (MEASURED 2026-10-14): the LIO publishes its registered scan as `/lio/cloud_world`, and the
planner's GridMap declares a subscription on `/cloud_registered` -- a topic that does not exist anywhere in
this stack (`ros2 topic list` with a fresh daemon: 14 topics, no `/cloud_registered`).  The planner was
therefore planning against an EMPTY map, which is why its plans skimmed 0.307 m from pillar surfaces while
the same scenario fed a synthetic cloud offline planned at 0.4978 m.

A `-r /cloud_registered:=/lio/cloud_world` remapping was added to the planner's command line first and the
node STILL reported its subscription as `/cloud_registered`, so this relay makes the name real instead of
depending on remapping semantics.  It is verifiable in one command: `ros2 topic hz /cloud_registered`.

usage: tools/cloud_relay.py [--in /lio/cloud_world] [--out /cloud_registered]
"""
from __future__ import annotations

import argparse
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="src", default="/lio/cloud_world")
    ap.add_argument("--out", dest="dst", default="/cloud_registered")
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import PointCloud2

    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    except Exception:
        rclpy.init(args=[])
    node = Node("cloud_relay")
    qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
    pub = node.create_publisher(PointCloud2, args.dst, qos)
    # RELIABLE_FOR_PLANNER: the planner subscribes with RELIABLE and a BEST_EFFORT publisher
    # cannot deliver to it (MEASURED 2026-10-14: "offering incompatible QoS. No messages will be
    # sent to it" -- the planner had NO map for every Mission B attempt until this line).
    qos_pub = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                          reliability=ReliabilityPolicy.RELIABLE)
    pub = node.create_publisher(PointCloud2, args.dst, qos_pub)
    state = {"n": 0, "t0": time.monotonic()}

    def on_cloud(m: PointCloud2):
        pub.publish(m)
        state["n"] += 1

    node.create_subscription(PointCloud2, args.src, on_cloud, qos)
    print("cloud_relay: %s -> %s" % (args.src, args.dst), flush=True)
    last = time.monotonic()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() - last > 5.0:                 # heartbeat: this tool must never fail silently
            last = time.monotonic()
            print("  relayed %d message(s) in %.0f s%s"
                  % (state["n"], time.monotonic() - state["t0"],
                     "" if state["n"] else "  (nothing arriving on %s)" % args.src), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
