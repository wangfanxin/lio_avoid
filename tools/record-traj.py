#!/usr/bin/env python3
"""Record every `/planner/traj` the FSM is offered, so the PLAN can be compared with the TRUTH afterwards.

WHY THIS EXISTS.  MEASURED 2026-10-14 on the 12-stop circle: C2 (the truth's own clearance) failed with
55.9 % of the samples inside the 0.20 m contract and 1.6 % inside a pillar, worst -0.349 m -- while the
adapter had published only paths it had verified to be >= 0.20 m clear.  Both statements can be true at
once only if the vehicle does not follow the plan exactly, and there was no record of the plan to measure
that against: the run dir held the truth (tap.csv), the LIO (lio.csv), the phase log and the FCU's BIN, and
nothing else.  "The gate cannot see the truth" was an argument; this file turns it into a number.

One row per message (the adapter republishes at 4 Hz, so a plan appears many times):
    t_us   the message's own stamp (system clock seconds -> us)
    t0_us  the trajectory's start stamp (what the FSM uses as the timeline)
    n      points in this message
    arc_m  the horizontal arc length of the published path
    clr_m  the path's own minimum clearance to the world file's pillars
    then one row per point in <csv>.points.csv, tagged with t_us so the two can be joined.

usage: tools/record-traj.py --csv out.csv --world worlds/forest_b.yaml [--topic /planner/traj] [--seconds N]
"""
from __future__ import annotations

import argparse
import importlib.util
import os
import sys
import time

import numpy as np


def load_world(path):
    """The adapter's own clearance definition, imported rather than re-implemented."""
    here = os.path.dirname(os.path.abspath(__file__))
    p = os.path.join(here, "..", "planner", "ego_adapter.py")
    spec = importlib.util.spec_from_file_location("ego_adapter_rt", p)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.World.load(path)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/planner/traj")
    ap.add_argument("--csv", required=True)
    ap.add_argument("--world", default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                    "..", "worlds", "forest_b.yaml"))
    ap.add_argument("--seconds", type=float, default=0.0, help="0 = until Ctrl-C")
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from nav_msgs.msg import Path

    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    except Exception:
        rclpy.init(args=[])
    world = load_world(args.world) if os.path.exists(args.world) else None
    node = Node("record_traj")
    # the adapter publishes this topic RELIABLE + TRANSIENT_LOCAL with DEPTH 1 (latched), so a subscriber
    # that is slower than the 4 Hz republish misses most of the messages -- MEASURED 2026-10-14: the first
    # recording held 56 of ~400 messages, and a plan that is 1.8 s stale is 1.8 m behind a vehicle flying at
    # 1 m/s, which is exactly the size of the "tracking error" it then reports.  A DEEPER queue on our side
    # cannot fix a depth-1 publisher, so the honest fix is (a) this queue, (b) reading fast, and (c) knowing
    # the limit: the join in tools/plan-vs-truth.py uses the message that was in force, and the count of
    # messages received versus the topic's own measured rate says how complete the recording is.
    qos = QoSProfile(depth=100, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    fh = open(args.csv, "w")
    fh.write("t_us,t0_us,n,arc_m,clr_m,z_min,z_max,frame_id\n")
    pf = open(args.csv + ".points.csv", "w")
    pf.write("t_us,i,e,n,u\n")
    state = {"n": 0}

    def on_path(m: Path):
        t = int(m.header.stamp.sec) * 1_000_000 + int(m.header.stamp.nanosec) // 1000
        if not len(m.poses):
            return
        p = np.array([[q.pose.position.x, q.pose.position.y, q.pose.position.z] for q in m.poses])
        t0 = int(m.poses[0].header.stamp.sec) * 1_000_000 + int(m.poses[0].header.stamp.nanosec) // 1000
        arc = float(np.sum(np.linalg.norm(np.diff(p[:, :2], axis=0), axis=1))) if len(p) > 1 else 0.0
        clr = float(world.clearance(p).min()) if world is not None else float("nan")
        fh.write("%d,%d,%d,%.6f,%.6f,%.4f,%.4f,%s\n"
                 % (t, t0, len(p), arc, clr, p[:, 2].min(), p[:, 2].max(), m.header.frame_id))
        for i, (x, y, z) in enumerate(p):
            pf.write("%d,%d,%.6f,%.6f,%.6f\n" % (t, i, x, y, z))
        fh.flush()
        pf.flush()
        state["n"] += 1

    node.create_subscription(Path, args.topic, on_path, qos)
    print("recording %s -> %s (world %s)" % (args.topic, args.csv, args.world), flush=True)
    t_start = time.monotonic()
    try:
        while args.seconds == 0.0 or (time.monotonic() - t_start) < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    fh.close()
    pf.close()
    print("recorded %d path messages" % state["n"])
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
