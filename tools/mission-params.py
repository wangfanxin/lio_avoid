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
           [--calib-file _stage/calib.json] [--advance-m 0.30] [--start-alt 1.28] [--dwell-at 0:10] [--loop]

--start-alt writes `fsm/waypoint_start_alt` (patch 0008's START-ALTITUDE GATE).  WHY IT EXISTS: MEASURED
earlier, asking ego to plan its FIRST trajectory while the vehicle is still on the pad left it in
GEN_NEW_TRAJ with 599 replans and 0 successes and the sortie had to be killed; the old external sequencer
worked around that by delaying its first goal until odom z >= max_stop_z - 0.60 = 1.28 m.  Without the gate
this script would re-introduce exactly that failure, because flight_type 2 makes ego start the tour in its
constructor.  The default -1.0 is DISABLED = the behaviour every earlier patch-0008 sortie flew.
"""
import argparse
import importlib.util
import inspect
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)


def load_world_module(mission="circle"):
    """The mission's stop list: `tools/world-circle.py` or `tools/world-rect.py`.

    Both are scripts with a hyphen in their name, so they are imported by path.  The mission shape is the
    OWNER'S decision: since 2026-10-15 the mission is the rectangle x in [-4, +4], y in [-3, +3], and the
    circle is kept only for comparison (it is no longer flown).
    """
    name = "world-rect.py" if mission == "rect" else "world-circle.py"
    path = os.path.join(ROOT, "tools", name)
    spec = importlib.util.spec_from_file_location("world_mission", path)
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
    ap.add_argument("--mission", default="rect", choices=["rect", "circle"],
                    help="the mission's stop list: 'rect' (the owner's current mission, x in [-4,+4], "
                         "y in [-3,+3]) or 'circle' (kept for comparison only)")
    ap.add_argument("--stops", type=int, default=0,
                    help="rect only: number of stops along the perimeter; 0 (the default) = the FOUR CORNERS, "
                         "which is the mission the owner asked for ('矩形只要四个角点')")
    ap.add_argument("--corners", action="store_true", help="rect only: force the 4 corners (already default)")
    ap.add_argument("--calib-file", default="/mnt/d/distill_self/_stage/calib.json",
                    help="the SAME L->W calibration the adapter and the FSM use")
    ap.add_argument("--out", default="", help="write the YAML here (default: stdout)")
    ap.add_argument("--advance-m", type=float, default=0.30,
                    help="the advance tolerance (AO2 MEASURED the aircraft settling 0.273 m from a commanded "
                         "stop, so upstream's no_replan_thresh_ = 0.15 m would never advance)")
    ap.add_argument("--dwell-at", default="", help="dwell seconds per stop index, e.g. '0:10' or '0:10;3:5'")
    ap.add_argument("--start-alt", type=float, default=-1.0,
                    help="fsm/waypoint_start_alt: > 0 = ego does NOT plan its first waypoint at "
                         "construction; the tour is armed by the FSM loop once odom z >= this.  -1.0 "
                         "(default) = disabled.  MEASURED earlier: planning from the pad gave 599 replans "
                         "and 0 successes, and the old external sequencer delayed its first goal until "
                         "max_stop_z - 0.60 = 1.28 m")
    ap.add_argument("--dwell-s", type=float, default=0.0, help="dwell for every stop not named in --dwell-at")
    ap.add_argument("--loop", action="store_true", help="loop the tour (default: stop after the last stop)")
    ap.add_argument("--node", default="ego_planner_node", help="the planner node name in the YAML")
    ap.add_argument("--stops-w-out", default="",
                    help="ALSO write the stops in W (one 'x,y,z' per line) here.  The stations in the YAML are "
                         "in L because ego works in L; the OBSERVER (planner/goal_sequencer.py, --observe-wp-index) "
                         "scores in W, and MEASURED sortie AP6-rect-dyn it scored the CIRCLE's twelve stops "
                         "while ego flew the owner's FOUR corners -- so every 'N of N' was against the wrong "
                         "list.  This file is the one list in both encodings, and run-mission.sh feeds it to "
                         "the sequencer as --stops=")
    args = ap.parse_args()

    wc = load_world_module(args.mission)
    calib = load_calib(args.calib_file)
    # Only pass the keywords the chosen generator actually accepts, so wiring one mission cannot break the
    # other (the rectangle takes n/corners, the circle takes neither).
    sig = inspect.signature(wc.commanded_points)
    kw = {}
    if "n" in sig.parameters and args.stops:
        kw["n"] = args.stops
    if "corners" in sig.parameters:
        # THE OWNER'S MISSION: the four corners of x in [-4,+4], y in [-3,+3].  Asking for perimeter stops
        # (--stops N) is the alternative, not the default.
        kw["corners"] = (args.stops <= 0) or args.corners
    stops_w = wc.commanded_points(args.world, **kw)
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

    lines = ["%s:" % args.node, "  ros__parameters:",
             "    # WRITTEN BY tools/mission-params.py -- the mission's stations as EGO'S OWN waypoint",
             "    # sequence (patch 0008).  Stops come from %s, converted once with %s."
             % (os.path.basename(args.world), os.path.basename(args.calib_file)),
             # FLAT, QUOTED, SLASH-SEPARATED KEYS.  NOT a nested `fsm:` map, and that is the whole point.
             # MEASURED 2026-10-15 (sortie AP1-stations, the first flight that ever passed this file to the
             # planner): with the nested map the file WAS delivered -- run-planner.sh's own header prints
             # `stations /home/fanxin/lio_avoid/logs/missionA-AP1-stations/stations.yaml (ego plans the
             # sequence itself)` -- and ego still came up with
             #   `WP-INDEX: sequence armed: -1 waypoints, advance 0.150 m, loop 0`
             # i.e. the DEFAULT of every key in this file.  ROS 2 flattens a nested map into DOT-separated
             # parameter names (`fsm.waypoint_num`), so the name the C++ declares, `fsm/waypoint_num`, kept
             # the value from the base params file (params/ego_planner_humble_coll20.yaml:12) and
             # `fsm/flight_type` stayed 1 = MANUAL_TARGET.  That second one is why the sortie could not even
             # reach patch 0008's gate: the gate is `target_type_ == PRESET_TARGET && waypoint_start_alt_ > 0`.
             # All three committed planner params files write this form (`"fsm/waypoint_num": -1`), so this
             # is the house convention and not a preference.
             '    "fsm/flight_type": 2                 # PRESET_TARGET',
             '    "fsm/waypoint_num": %d' % len(stops_l),
             '    "fsm/waypoint_advance_m": %.3f' % args.advance_m,
             # patch 0008's START-ALTITUDE GATE: the tour must not be armed while the vehicle is on the pad.
             '    "fsm/waypoint_start_alt": %.2f' % args.start_alt,
             '    "fsm/waypoint_loop": %s' % ("true" if args.loop else "false")]
    for i, q in enumerate(stops_l):
        lines.append('    "fsm/waypoint%d_x": %.4f' % (i, q[0]))
        lines.append('    "fsm/waypoint%d_y": %.4f' % (i, q[1]))
        lines.append('    "fsm/waypoint%d_z": %.4f' % (i, q[2]))
    for i in range(len(stops_l)):
        lines.append('    "fsm/waypoint%d_dwell_s": %.1f' % (i, dwell.get(i, args.dwell_s)))
    text = "\n".join(lines) + "\n"

    if args.out:
        with open(args.out, "w") as fh:
            fh.write(text)

    if args.stops_w_out:
        with open(args.stops_w_out, "w") as fh:
            for p in stops_w:
                fh.write("%.4f,%.4f,%.4f\n" % (float(p[0]), float(p[1]), float(p[2])))
        print("   the SAME %d stops in W (for the observer's --stops=): %s"
              % (len(stops_w), args.stops_w_out))
    print("mission-params: mission=%s, %d stops from %s -> %s"
          % (args.mission, len(stops_l), os.path.basename(args.world), args.out or "stdout"))
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
