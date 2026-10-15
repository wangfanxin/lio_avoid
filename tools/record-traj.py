#!/usr/bin/env python3
"""Record every plan the trajectory source publishes, so the PLAN can be compared with the TRUTH afterwards.

WHY THIS EXISTS.  MEASURED 2026-10-14 on a 12-stop circle flight: C2 (the truth's own clearance) failed with
55.9 % of the samples inside the 0.20 m contract and 1.6 % inside a pillar, worst -0.349 m -- while the
source had published only paths verified to be >= 0.20 m clear.  Both statements can be true at
once only if the vehicle does not follow the plan exactly, and there was no record of the plan to measure
that against: the run dir held the truth (tap.csv), the LIO (lio.csv), the phase log and the FCU's BIN, and
nothing else.  "The gate cannot see the truth" was an argument; this file turns it into a number.

F1 (2026-10-16) MOVED IT TO THE COMMAND STREAM, AND WHY THAT HAD TO HAPPEN IN THE SAME CHANGE.  The plan box
no longer publishes `nav_msgs/Path`; it publishes `quadrotor_msgs/msg/PositionCommand` on `/plan/cmd`
(design risk R5: "if it is not moved in the same change, every run loses its plan-clearance column and
nobody notices until an audit").  `--msg position_cmd` subscribes to that instead, and the RECORDING KEEPS
THE SAME MEANING, which is the point:

  * a command stream is a sequence of WAYPOINTS, so the plan's geometry between them is the STRAIGHT LINE
    from the last distinct commanded point to this one.  `arc_m`, `clr_m` and `z_min/max` are computed over
    that segment, so `clr_m` still means "the plan's minimum pillar clearance at this instant" and not
    "the clearance of one corner";
  * the points file gets a row only when the commanded point CHANGES, so it still holds one polyline per
    LEG, tagged with the leg's own start stamp (`t0_us`) -- which is what `tools/plot-run.py` groups by
    ("one line per distinct PLAN, not per message") and what `tools/plan-vs-truth.py` joins against the
    truth.

One row per message (a source may republish, so one plan appears many times):
    t_us   the message's own stamp (system clock seconds -> us)
    t0_us  the trajectory's start stamp (the first message of this leg/plan)
    n      points in this message (1 for a command stream)
    arc_m  the horizontal arc length of the geometry this message commands (frame-independent)
    clr_m  its minimum clearance to the world file's pillars, IN W
    z_min_w, z_max_w   its vertical extent, IN W
    frame_id  what the MESSAGE said -- the wire frame, `lio_world` = L
    then one row per DISTINCT commanded point in <csv>.points.csv, tagged with t_us and its frame.

usage: tools/record-traj.py --csv out.csv --world worlds/pillars.yaml --calib-file _stage/calib.json
       [--msg path|position_cmd] [--topic /plan/cmd] [--seconds N]
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

#: how finely the straight line between two commanded waypoints is sampled when its clearance is measured.
#: 20 mm is 25x finer than the 0.20 m contract it is checked against, and a 2.0 m leg costs 100 points.
SEG_SAMPLES = 101


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
    ap.add_argument("--topic", default="/plan/cmd")
    ap.add_argument("--msg", default="position_cmd", choices=["path", "position_cmd"],
                    help="what the source publishes: `position_cmd` is the PLAN box's command stream "
                         "(`quadrotor_msgs/msg/PositionCommand`, the interface since F1); `path` is the "
                         "old `nav_msgs/Path`, kept so an OLD run can be re-recorded and so the change is "
                         "reversible")
    ap.add_argument("--csv", required=True)
    ap.add_argument("--world", default=os.path.join(ROOT, "worlds", "pillars.yaml"))
    ap.add_argument("--calib-file", default="",
                    help="the MEASURED L->W calibration -- the SAME file the FSM and the producer use.  "
                         "Without it the plan cannot be placed in the world the pillars live in, so clr_m "
                         "is written as nan and said out loud")
    ap.add_argument("--seconds", type=float, default=0.0, help="0 = until Ctrl-C")
    args = ap.parse_args()
    if args.topic == "/planner/traj" and args.msg == "position_cmd":
        ap.error("--topic /planner/traj with --msg position_cmd: the Path interface is gone (F1); pass "
                 "--msg path if you really mean to record a Path")

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

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
    # the producer publishes RELIABLE + TRANSIENT_LOCAL, so a subscriber that is slower than the stream
    # misses messages -- MEASURED 2026-10-14: the first recording held 56 of ~400 messages, and a plan that
    # is 1.8 s stale is 1.8 m behind a vehicle flying at 1 m/s, which is exactly the size of the "tracking
    # error" it then reports.  A DEEPER queue on our side cannot fix a depth-1 publisher, so the honest fix
    # is (a) this queue, (b) reading fast, and (c) knowing the limit.
    qos = QoSProfile(depth=100, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    fh = open(args.csv, "w")
    fh.write("t_us,t0_us,n,arc_m,clr_m,z_min_w,z_max_w,frame_id\n")
    pf = open(args.csv + ".points.csv", "w")
    pf.write("t_us,i,e,n,u,frame\n")
    # THE REFERENCE AT THE RATE IT IS ACTUALLY PUBLISHED -- one row per MESSAGE, no de-duplication.
    #
    # WHY A SECOND FILE: `traj.csv.points.csv` deliberately holds one point per DISTINCT commanded
    # position, because that is what the readers written against the old `nav_msgs/Path` need (one
    # polyline per leg, grouped by `t0_us`).  MEASURED (and reported by the delegating agent on RECT24):
    # that file therefore records 405 points for 6220 messages = 8.4/s, in a 20 mm raster, so "is the
    # REFERENCE itself smooth, or is the aircraft failing to track a smooth reference?" -- the first
    # question anyone asks about a crooked flight -- was unanswerable from the files a run keeps.
    # This file answers it: every message, every field, in L.
    cf = open(args.csv + ".cmd.csv", "w")
    cf.write("t_us,x_l,y_l,z_l,vx_l,vy_l,vz_l,ax_l,ay_l,az_l,yaw,yaw_dot,trajectory_id,"
             "trajectory_flag,frame_id\n")
    state = {"n": 0, "pts": 0, "last": None, "t0": None, "msgs": 0}

    print("recording %s (%s) -> %s (world %s)" % (args.topic, args.msg, args.csv, args.world), flush=True)
    if world is None:
        print("  NO WORLD FILE at %s: clr_m will be nan", args.world, flush=True)
    if calib is None:
        print("  NO --calib-file: the plan arrives in L and the world is in W, so the clearance CANNOT be "
              "computed and clr_m will be nan (this is the state that produced -0.2972 m for a plan that is "
              "0.6000 m clear)", flush=True)
    else:
        print("  L->W from %s: %s -- every geometric column is written in W" % (args.calib_file,
                                                                                calib.summary()), flush=True)

    def emit(t_us, t0_us, pts_l, frame, distinct=False):
        """One row: the geometry (a polyline, or the segment a command implies) in L, written in W."""
        p = np.asarray(pts_l, dtype=float).reshape(-1, 3)
        arc = float(np.sum(np.linalg.norm(np.diff(p[:, :2], axis=0), axis=1))) if len(p) > 1 else 0.0
        # THE FRAME THE WORLD IS IN.  `p` is what the message carried (L, `frame_id = lio_world`); the
        # pillars are in W.  One rotation, applied once, here -- so `clr_m` means what its name says.
        pw = (calib.R_WL @ p.T).T + calib.t_WL if calib is not None else None
        clr = float(world.clearance(pw).min()) if (world is not None and pw is not None) else float("nan")
        zmin = float(pw[:, 2].min()) if pw is not None else float("nan")
        zmax = float(pw[:, 2].max()) if pw is not None else float("nan")
        fh.write("%d,%d,%d,%.6f,%.6f,%.4f,%.4f,%s\n"
                 % (t_us, t0_us, len(p), arc, clr, zmin, zmax, frame))
        if distinct:
            for i, (x, y, z) in enumerate(pw if pw is not None else p):
                pf.write("%d,%d,%.6f,%.6f,%.6f,%s\n" % (t_us, i, x, y, z,
                                                        "W" if pw is not None else "L"))
                state["pts"] += 1
        fh.flush()
        pf.flush()
        state["n"] += 1

    if args.msg == "path":
        from nav_msgs.msg import Path

        def on_path(m: Path):
            t = int(m.header.stamp.sec) * 1_000_000 + int(m.header.stamp.nanosec) // 1000
            if not len(m.poses):
                return
            p = np.array([[q.pose.position.x, q.pose.position.y, q.pose.position.z] for q in m.poses])
            t0 = int(m.poses[0].header.stamp.sec) * 1_000_000 + int(m.poses[0].header.stamp.nanosec) // 1000
            emit(t, t0, p, m.header.frame_id, distinct=True)

        node.create_subscription(Path, args.topic, on_path, qos)
    else:
        from quadrotor_msgs.msg import PositionCommand

        def on_cmd(m: PositionCommand):
            """THE COMMAND STREAM.  The plan is the SEQUENCE of commanded points, so the geometry a message
            implies is the straight line from the last DISTINCT commanded point to this one (a point, while
            the command does not change) -- and the points file gets a row only when it changes, which
            keeps `traj.csv` a per-LEG polyline for the readers that were written against the Path.
            """
            t = int(m.header.stamp.sec) * 1_000_000 + int(m.header.stamp.nanosec) // 1000
            p = np.array([m.position.x, m.position.y, m.position.z], dtype=float)
            cf.write("%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%s\n"
                     % (t, p[0], p[1], p[2],
                        m.velocity.x, m.velocity.y, m.velocity.z,
                        m.acceleration.x, m.acceleration.y, m.acceleration.z,
                        m.yaw, m.yaw_dot, m.trajectory_id, m.trajectory_flag, m.header.frame_id))
            cf.flush()
            state["msgs"] += 1
            prev = state["last"]
            changed = prev is None or float(np.linalg.norm(p - prev)) > 1e-9
            if changed:
                state["last"] = p
                state["t0"] = t
                seg = p.reshape(1, 3) if prev is None else np.linspace(prev, p, SEG_SAMPLES)
                emit(t, state["t0"], seg, m.header.frame_id, distinct=True)
            else:
                emit(t, state["t0"], p.reshape(1, 3), m.header.frame_id, distinct=False)

        node.create_subscription(PositionCommand, args.topic, on_cmd, qos)

    t_start = time.monotonic()
    try:
        while args.seconds == 0.0 or (time.monotonic() - t_start) < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    fh.close()
    pf.close()
    cf.close()
    print("recorded %d messages (%d distinct commanded points); the reference at its own rate is in %s"
          % (state["n"], state["pts"], args.csv + ".cmd.csv"))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
