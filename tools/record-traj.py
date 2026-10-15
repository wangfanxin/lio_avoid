#!/usr/bin/env python3
"""Record every `/planner/traj` the FSM is offered, so the PLAN can be compared with the TRUTH afterwards.

WHY THIS EXISTS.  MEASURED 2026-10-14 on a 12-stop circle flight: C2 (the truth's own clearance) failed with
55.9 % of the samples inside the 0.20 m contract and 1.6 % inside a pillar, worst -0.349 m -- while the
source had published only paths verified to be >= 0.20 m clear.  Both statements can be true at
once only if the vehicle does not follow the plan exactly, and there was no record of the plan to measure
that against: the run dir held the truth (tap.csv), the LIO (lio.csv), the phase log and the FCU's BIN, and
nothing else.  "The gate cannot see the truth" was an argument; this file turns it into a number.

One row per message (a source may republish, so one plan appears many times):
    t_us   the message's own stamp (system clock seconds -> us)
    t0_us  the trajectory's start stamp (what the FSM uses as the timeline)
    n      points in this message
    arc_m  the horizontal arc length of the published path (frame-independent)
    clr_m  the path's own minimum clearance to the world file's pillars, IN W
    z_min_w, z_max_w   the path's vertical extent, IN W
    frame_id  what the MESSAGE said -- the wire frame, `lio_world` = L
    then one row per point in <csv>.points.csv, tagged with t_us and with the frame it is in.

THE FRAME, AND WHY THE POINTS FILE NOW CARRIES IT (2026-10-15).  The Path arrives in L and the world file is
in W, and this recorder used to hand the L-frame points straight to the W-frame clearance function:
MEASURED on `logs/missionA-RECT6-clean-square`, `clr_m` read **-0.2972 m** for a plan whose real clearance is
**+0.6000 m** (the world's designed worst case).  A column called `clr_m` that is 0.9 m wrong is worse than
no column: it is a number someone will quote.  So the recorder now applies the SAME `--calib-file` the FSM
and the producer use (L -> W), writes every geometric column in W, and records `frame` on every point row --
and if no calibration is given it writes `nan` and says so loudly, because a clearance computed in the wrong
frame is not a clearance.  C2 was never affected: it is measured on the flight's own truth inside
`tools/metrics.py`, and no acceptance number has ever come from this column.

usage: tools/record-traj.py --csv out.csv --world worlds/pillars.yaml --calib-file _stage/calib.json
       [--topic /planner/traj] [--seconds N]
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)


def load_world(path):
    """The pillar clearance, IMPORTED rather than re-implemented.

    It used to come from `planner/ego_adapter.py`; that file was deleted with the rest of the dropped
    ego-planner line on 2026-10-15, and the same finite-cylinder rule now lives in the one file that
    reads the world file (`tools/world-info.py::PillarWorld`).
    """
    p = os.path.join(HERE, "world-info.py")
    spec = importlib.util.spec_from_file_location("world_info_rt", p)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.PillarWorld.load(path)


def load_calib(path):
    """The MEASURED L -> W calibration, the same file and the same class the FSM and the producer use.

    Imported as a MODULE, not by file path: `common/frames.py` defines a dataclass, and a dataclass built
    from a module importlib never registered in `sys.modules` dies inside dataclasses itself (measured
    2026-10-14 by tools/plan-vs-truth.py, which fell back to "no plans drawn" and hid it).
    """
    from common import frames
    with open(path) as fh:
        d = json.load(fh)
    return frames.Calib(R_WL=np.array(d["R_WL"], dtype=float), t_WL=np.array(d["t_WL"], dtype=float))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/planner/traj")
    ap.add_argument("--csv", required=True)
    ap.add_argument("--world", default=os.path.join(ROOT, "worlds", "pillars.yaml"))
    ap.add_argument("--calib-file", default="",
                    help="the MEASURED L->W calibration -- the SAME file the FSM and the producer use.  "
                         "Without it the path cannot be placed in the world the pillars live in, so clr_m "
                         "is written as nan and said out loud")
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
    calib = None
    if args.calib_file and os.path.exists(args.calib_file):
        calib = load_calib(args.calib_file)
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
    fh.write("t_us,t0_us,n,arc_m,clr_m,z_min_w,z_max_w,frame_id\n")
    pf = open(args.csv + ".points.csv", "w")
    pf.write("t_us,i,e,n,u,frame\n")
    state = {"n": 0}

    print("recording %s -> %s (world %s)" % (args.topic, args.csv, args.world), flush=True)
    if world is None:
        print("  NO WORLD FILE at %s: clr_m will be nan", args.world, flush=True)
    if calib is None:
        print("  NO --calib-file: the path arrives in L and the world is in W, so the clearance CANNOT be "
              "computed and clr_m will be nan (this is the state that produced -0.2972 m for a plan that is "
              "0.6000 m clear)", flush=True)
    else:
        print("  L->W from %s: %s -- every geometric column is written in W" % (args.calib_file,
                                                                                calib.summary()), flush=True)

    def on_path(m: Path):
        t = int(m.header.stamp.sec) * 1_000_000 + int(m.header.stamp.nanosec) // 1000
        if not len(m.poses):
            return
        p = np.array([[q.pose.position.x, q.pose.position.y, q.pose.position.z] for q in m.poses])
        t0 = int(m.poses[0].header.stamp.sec) * 1_000_000 + int(m.poses[0].header.stamp.nanosec) // 1000
        arc = float(np.sum(np.linalg.norm(np.diff(p[:, :2], axis=0), axis=1))) if len(p) > 1 else 0.0
        # THE FRAME THE WORLD IS IN.  `p` is what the message carried (L, `frame_id = lio_world`); the
        # pillars are in W.  One rotation, applied once, here -- so `clr_m` means what its name says.
        pw = (calib.R_WL @ p.T).T + calib.t_WL if calib is not None else None
        clr = float(world.clearance(pw).min()) if (world is not None and pw is not None) else float("nan")
        zmin = float(pw[:, 2].min()) if pw is not None else float("nan")
        zmax = float(pw[:, 2].max()) if pw is not None else float("nan")
        fh.write("%d,%d,%d,%.6f,%.6f,%.4f,%.4f,%s\n"
                 % (t, t0, len(p), arc, clr, zmin, zmax, m.header.frame_id))
        for i, (x, y, z) in enumerate(pw if pw is not None else p):
            pf.write("%d,%d,%.6f,%.6f,%.6f,%s\n" % (t, i, x, y, z, "W" if pw is not None else "L"))
        fh.flush()
        pf.flush()
        state["n"] += 1

    node.create_subscription(Path, args.topic, on_path, qos)
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
