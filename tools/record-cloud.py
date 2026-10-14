#!/usr/bin/env python3
"""Record a PointCloud2 to a compact CSV (x,y,z only), so the map the planner saw can be compared with
the world file afterwards.

WHY: MEASURED 2026-10-14, Mission B's last link.  The adapter withholds e.g. every path with
"pillar clearance 0.307 m < footprint 0.35 m", while the SAME scenario run offline -- goals and start
identical, cloud synthesised from the world file by sim/raycast.py -- plans at 0.4978 m.  The planner can
therefore hold the margin; what differs is the MAP.  This recording is what turns "the live map differs"
into a number: cast the recorded cloud against the world's cylinders and see which pillars are missing,
short, or displaced.

usage: tools/record-cloud.py --topic /cloud_registered --csv out.csv [--seconds N] [--decimate K]
"""
from __future__ import annotations

import argparse
import struct
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/cloud_registered")
    ap.add_argument("--csv", required=True)
    ap.add_argument("--seconds", type=float, default=0.0)
    ap.add_argument("--decimate", type=int, default=20, help="keep every K-th point")
    ap.add_argument("--scans", type=int, default=3, help="how many scans to record")
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
    node = Node("record_cloud")
    qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
    fh = open(args.csv, "w")
    fh.write("scan,t_us,x,y,z\n")
    state = {"n": 0, "k": 0, "t0": time.monotonic()}

    def on_cloud(m: PointCloud2):
        state["k"] += 1
        stamp = int(m.header.stamp.sec) * 1_000_000 + int(m.header.stamp.nanosec) // 1000
        raw = bytes(m.data)
        step = m.point_step
        for i in range(0, m.width * m.height, max(1, args.decimate)):
            off = i * step
            x, y, z = struct.unpack_from("<fff", raw, off)
            if x == x and y == y and z == z:            # drop NaNs
                fh.write("%d,%d,%.4f,%.4f,%.4f\n" % (state["k"], stamp, x, y, z))
        state["n"] += 1

    node.create_subscription(PointCloud2, args.topic, on_cloud, qos)
    print("recording %s -> %s (every %d-th point, %d scans)" % (args.topic, args.csv, args.decimate,
                                                                args.scans), flush=True)
    last_beat = time.monotonic()
    while rclpy.ok() and state["n"] < args.scans:
        if args.seconds and time.monotonic() - state["t0"] > args.seconds:
            break
        rclpy.spin_once(node, timeout_sec=0.2)
        # HEARTBEAT.  MEASURED 2026-10-14: this tool failed by printing NOTHING (it only printed after N
        # scans), which cost a round of guessing -- three tools in this project have now failed that way.
        # Say what is happening: how long we have waited, how many messages arrived, and what the graph
        # looks like from here.
        if time.monotonic() - last_beat > 3.0:
            last_beat = time.monotonic()
            topics = [t for t, _ in node.get_topic_names_and_types()]
            print("  waiting %.0f s: %d messages, %d topic(s) visible%s"
                  % (time.monotonic() - state["t0"], state["n"], len(topics),
                     "" if args.topic in topics else "  (NOT publishing %s from here)" % args.topic),
                  flush=True)
    fh.close()
    print("recorded %d scans, %d points" % (state["n"], state["k"]))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if state["n"] else 1


if __name__ == "__main__":
    sys.exit(main())
