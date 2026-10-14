#!/usr/bin/env python3
"""Write ego's station list as a params file: the mission's stops become EGO'S OWN waypoint sequence.

WHY THIS EXISTS.  Until now the stop list lived in `planner/goal_sequencer.py`, which pushed one goal at a
time through `planner/ego_adapter.py` into ego, and judged "reached" itself.  That made two sources of truth
for "where the mission is", and MEASURED sortie AO2 showed the cost: the adapter had converted the
sequencer's next goal (W[+1.5918 +3.6696] -> L[+3.6723 -1.5958], round trip 0.000000 mm) while ego's own
`end_pt_` was still the previous stop's [0.001 -4.001 1.884].  Upstream ego-planner has the sequence
built in -- `fsm/flight_type=2` (PRESET_TARGET) + `fsm/waypoint_num` + `fsm/waypointN_x/y/z` +
`planNextWaypoint()` -> `planGlobalTraj(odom -> waypoint)` -- and its advance test is a test on the VEHICLE
(`(end_pt_ - pos).norm() < threshold`), so the sequence structurally cannot run ahead of the aircraft.
Patch 0008 (patches/0008-ego-waypoint-sequence.patch) adds the three things upstream's version lacks for
these missions (`fsm/waypoint_advance_m`, `fsm/waypointN_dwell_s`, `fsm/waypoint_loop=false`) and publishes
`wp_index` so the scoring layer OBSERVES the sequence.

The stops themselves are NOT redefined here: they come from the same frozen world file the sequencer reads
(`tools/world-circle.py::commanded_points(world)`), so this script only changes WHERE the sequence lives,
never what it is.  The one transformation is the frame: the stops are written in the world's own frame
(world_w, the pad) and ego works in the LIO's frame (L), so every stop is converted ONCE, here, with the
measured calibration (`planner/common/frames.py`), instead of per goal at flight time.

usage: tools/mission-params.py --world worlds/forest_b.yaml --out /tmp/stations.yaml \
           [--calib-file _stage/calib.json] [--advance-m 0.30] [--dwell-at 0:10] [--loop]
"""
import argparse
import importlib.util
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)


def load_world_module():
    """tools/world-circle.py is a script with a hyphen in its name, so import it by path."""
    path = os.path.join(ROOT, "tools", "world-circle.py")
    spec = importlib.util.spec_from_file_location("world_circle", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def load_calib(path):
    from common import frames
    if not os.path.exists(path):
        raise SystemExit("no calibration at %s -- the station list must be converted with the SAME measured "
                         "calibration the adapter and the FSM use; refusing to guess one" % path)
    with open(path) as fh:
        d = json.load(fh)
    return frames.Calib(R_WL=np.array(d["R_WL"], dtype=float),
                        t_WL=np.array(d["t_WL"], dtype=float),
                        n_samples=int(d.get("n_samples", 0)),
                        rms_m=float(d.get("rms_m", float("nan"))))


def w_to_l(calib, p_w):
    """W -> L, the same expression the adapter uses (`planner/ego_adapter.py:233`)."""
    return calib.R_WL.T @ (np.asarray(p_w, dtype=float) - calib.t_WL)


def l_to_w(calib, p_l):
    return calib.R_WL @ np.asarray(p_l, dtype=float) + calib.t_WL


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default=os.path.join(ROOT, "worlds", "forest_b.yaml"))
    ap.add_argument("--calib-file", default="/mnt/d/distill_self/_stage/calib.json",
                    help="the SAME L->W calibration the adapter and the FSM use")
    ap.add_argument("--out", default="", help="write the YAML here (default: stdout)")
    ap.add_argument("--advance-m", type=float, default=0.30,
                    help="the advance tolerance (AO2 MEASURED the aircraft settling 0.273 m from a commanded "
                         "stop, so upstream's no_replan_thresh_ = 0.15 m would never advance)")
    ap.add_argument("--dwell-at", default="", help="dwell seconds per stop index, e.g. '0:10' or '0:10;3:5'")
    ap.add_argument("--dwell-s", type=float, default=0.0, help="dwell for every stop not named in --dwell-at")
    ap.add_argument("--loop", action="store_true", help="loop the tour (default: stop after the last stop)")
    ap.add_argument("--node", default="ego_planner_node", help="the planner node name in the YAML")
    args = ap.parse_args()

    wc = load_world_module()
    calib = load_calib(args.calib_file)
    stops_w = wc.commanded_points(args.world)
    if not stops_w:
        print("no commanded points in %s" % args.world, file=sys.stderr)
        return 1

    dwell = {}
    for spec in args.dwell_at.split(";"):
        if spec.strip():
            i, s = spec.split(":")
            dwell[int(i)] = float(s)

    stops_l = [w_to_l(calib, p) for p in stops_w]
    # the round trip is the check that cannot pass by accident: it uses the inverse on the same data
    rt = max(float(np.linalg.norm(l_to_w(calib, q) - np.asarray(p, dtype=float)))
             for p, q in zip(stops_w, stops_l))

    lines = ["%s:" % args.node, "  ros__parameters:", "    fsm:",
             "      # WRITTEN BY tools/mission-params.py -- the mission's stations as EGO'S OWN waypoint",
             "      # sequence (patch 0008).  Stops come from %s, converted once with %s."
             % (os.path.basename(args.world), os.path.basename(args.calib_file)),
             "      flight_type: 2                 # PRESET_TARGET",
             "      waypoint_num: %d" % len(stops_l),
             "      waypoint_advance_m: %.3f" % args.advance_m,
             "      waypoint_loop: %s" % ("true" if args.loop else "false")]
    for i, q in enumerate(stops_l):
        lines.append("      waypoint%d_x: %.4f" % (i, q[0]))
        lines.append("      waypoint%d_y: %.4f" % (i, q[1]))
        lines.append("      waypoint%d_z: %.4f" % (i, q[2]))
    for i in range(len(stops_l)):
        lines.append("      waypoint%d_dwell_s: %.1f" % (i, dwell.get(i, args.dwell_s)))
    text = "\n".join(lines) + "\n"

    if args.out:
        with open(args.out, "w") as fh:
            fh.write(text)

    print("mission-params: %d stops from %s -> %s" % (len(stops_l), os.path.basename(args.world),
                                                      args.out or "stdout"))
    print("   calibration  %s" % calib.summary())
    print("   advance %.3f m, loop %s, dwell %s" % (args.advance_m, args.loop,
                                                   dwell if dwell else args.dwell_s))
    print("   round trip W->L->W: %.6f mm (must be ~0)" % (rt * 1000.0))
    for i, (p, q) in enumerate(zip(stops_w, stops_l)):
        print("   stop %2d  W[%+8.4f %+8.4f %+6.3f] -> L[%+8.4f %+8.4f %+6.3f]  dwell %.1f s"
              % (i, p[0], p[1], p[2], q[0], q[1], q[2], dwell.get(i, args.dwell_s)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
