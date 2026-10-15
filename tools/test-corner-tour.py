#!/usr/bin/env python3
"""The corner-tour contract, tested on synthetic message sequences -- no flight, no ROS, no SITL.

WHY THIS EXISTS (2026-10-15).  With the corner hold ON, `planner/rectangle_source.py` publishes ONE LEG PER
MESSAGE and advances ON ARRIVAL, and `fsm/mission_fsm.py::legs()` has to fly the WHOLE square from those
messages.  Whether it does is a question about two pieces of the FSM's own code -- what it detects as a
corner in the path in hand, and which of those corners it still has to fly -- and both can be asked
directly, with the real methods, on the messages the producer would publish.  A flight is the wrong place to
find out that a shared corner became a zero-length leg.

WHAT IT ASSERTS:
  1. the four legs of the tour, fed one message at a time, produce **5 leg phases**: the entry leg
     (pad -> corner 1, which the FIRST message's corner list already contains) and the four square legs.
     The corner shared between consecutive messages is dropped, because the vehicle is parked on it;
  2. NEGATIVE CONTROL: the same four messages WITHOUT the drop rule ask for 8 corners (each open leg is
     detected as its two ends), i.e. 3 legs that measure nothing -- which is what the guard prevents;
  3. a CLOSED tour (RECT1/RECT6's shape) is detected as 5 corners and is flown as 5 legs, and `_path_in_hand`
     calls it closed -- so that configuration's behaviour is the one it always was;
  4. an EMPTY Path sets the "no more legs" flag, which is how the consumer ends a tour on a statement.

usage: tools/test-corner-tour.py [--world worlds/pillars.yaml]
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from common import frames                       # noqa: E402
from fsm.mission_fsm import Mission, _corners_of   # noqa: E402
from planner.rectangle_source import densify    # noqa: E402


def read_world(path):
    import importlib.util
    spec = importlib.util.spec_from_file_location("world_info_tour", os.path.join(HERE, "world-info.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.read(path)


class _Pos:
    __slots__ = ("x", "y", "z")

    def __init__(self, p):
        self.x, self.y, self.z = float(p[0]), float(p[1]), float(p[2])


class _Pose:
    __slots__ = ("position", "orientation")

    def __init__(self, p):
        self.position = _Pos(p)
        self.orientation = None


class _Stamp:
    __slots__ = ("sec", "nanosec")

    def __init__(self, t):
        self.sec = int(t)
        self.nanosec = int((t - int(t)) * 1e9)


class _Hdr:
    __slots__ = ("stamp", "frame_id")

    def __init__(self, t):
        self.stamp = _Stamp(t)
        self.frame_id = "lio_world"


class _PoseStamped:
    __slots__ = ("pose", "header")

    def __init__(self, p, t):
        self.pose = _Pose(p)
        self.header = _Hdr(t)


class _Path:
    """Enough of a nav_msgs/Path for `_on_path`: a frame_id and stamped poses."""

    def __init__(self, pts, t=0.0):
        self.header = _Hdr(t)
        self.poses = [_PoseStamped(p, t) for p in pts]


class _State:
    def __init__(self, pos):
        self.pos = np.asarray(pos, dtype=float)


def make_mission(calib, pos):
    """A Mission with everything `_on_path`/`_path_in_hand`/`_legs_left` touch, and nothing else."""
    m = Mission.__new__(Mission)                     # no bridge, no ROS node, no logs
    m.calib = calib
    m.args = argparse.Namespace(closed_m=0.05)
    m.path_l, m.path_t, m.path_ident = None, None, None
    m.paths_seen, m.last_path_wall, m.traj_last_empty = 0, 0.0, False
    m.args.leg_join_m = 0.25
    bridge = type("B", (), {})()
    bridge.state = lambda: _State(pos)
    m.bridge = bridge
    return m


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default=os.path.join(ROOT, "worlds", "pillars.yaml"))
    ap.add_argument("--spacing", type=float, default=0.05)
    args = ap.parse_args()
    problems = []

    world = read_world(args.world)
    c = np.asarray([[q[0], q[1], q[2]] for q in world["corners"]], dtype=float)
    if len(c) > 1 and np.allclose(c[0], c[-1]):
        c = c[:-1]
    n_c = len(c)
    print("world %s: %d mission corners" % (os.path.basename(args.world), n_c))

    # the producer's own conversions: it publishes W -> L and this FSM converts L -> F with the same calib
    calib = frames.Calib.predicted(spawn_heading_deg=0.0, world_axes="body")
    R_LW = calib.R_WL.T

    def to_l(p_w):
        return (R_LW @ (np.asarray(p_w, dtype=float) - calib.t_WL).T).T

    # ---- 1/2. the tour, one message per leg, and the negative control --------------------------------
    legs_w = [np.linspace(c[i], c[(i + 1) % n_c], max(2, int(float(np.linalg.norm(c[(i + 1) % n_c] - c[i]))
                                                            / args.spacing) + 1))
              for i in range(n_c)]
    pos = np.zeros(3)                                # the vehicle starts on the pad
    m = make_mission(calib, pos)
    flown, no_drop = [], []
    print()
    print("the tour as the source publishes it (hold ON: one OPEN leg per message, advance on arrival):")
    for i, leg in enumerate(legs_w):
        # WHERE THE VEHICLE IS when this message is in hand: on the PAD for the first one (the FSM enters
        # `legs()` after the takeoff hold, with leg 1 already latched) and parked on the previous message's
        # end corner for every later one -- that shared corner is the one the drop rule removes.
        before = np.zeros(3) if i == 0 else np.asarray(c[i], dtype=float)
        m.bridge.state = lambda pos=before: _State(pos)
        m._on_path(_Path(to_l(leg), t=float(i)))
        path_f, curve, closed = m._path_in_hand()
        keep = m._legs_left(curve)
        no_drop += curve
        flown += keep
        print("   message %d: leg corner %d -> %d, %d points | detected %d corners, %d still to fly: %s"
              % (i + 1, i, (i + 1) % n_c, len(path_f), len(curve), len(keep),
                 " ".join("(E %+.3f N %+.3f)" % (q[0], q[1]) for q in keep)))
        if closed:
            problems.append("message %d (an open leg) was classified as a CLOSED tour" % (i + 1))
        if i == 0 and len(keep) != 2:
            problems.append("the first message should ask for 2 corners (the entry leg pad -> corner 1 and "
                            "then corner 2), got %d" % len(keep))
        if i > 0 and len(keep) != 1:
            problems.append("message %d should ask for 1 corner (the shared corner is behind us), got %d"
                            % (i + 1, len(keep)))
        if i > 0 and len(keep) == 1 and float(np.linalg.norm(keep[0][:2] - c[(i + 1) % n_c][:2])) > 1e-6:
            problems.append("message %d's remaining corner is not the leg's end" % (i + 1))
        # and after flying it, the vehicle is parked on the leg's end -- which is the next message's start
        m.bridge.state = lambda pos=np.asarray(c[(i + 1) % n_c], dtype=float): _State(pos)
    print("   -> %d leg phases for the whole square" % len(flown))
    if len(flown) != n_c + 1:
        problems.append("the tour produced %d leg phases, expected %d (the entry leg plus the %d square "
                        "legs): B2's per-leg table is the evidence the acceptance reads"
                        % (len(flown), n_c + 1, n_c))
    print("   NEGATIVE CONTROL -- the same messages WITHOUT dropping the reached corner:")
    print("      %d corners would be visited (each open leg reads as its two ends), i.e. %d legs that "
          "measure nothing" % (len(no_drop), len(no_drop) - len(flown)))
    if len(no_drop) != 2 * n_c:
        problems.append("the negative control expected %d corners without the drop rule, got %d"
                        % (2 * n_c, len(no_drop)))

    # ---- 2b. the drop rule against a STANDING ERROR, and its limit ------------------------------------
    print()
    print("the shared corner (message 2's start = message 1's end) against a standing error -- the position")
    print("loop leaves the aircraft 0.005-0.03 m short of its command (C1 measured 0.031 m):")
    for off, want in ((0.0, 1), (0.031, 1), (0.12, 1), (0.24, 1), (0.60, 2)):
        m = make_mission(calib, c[1] + np.array([off, 0.0, 0.0]))
        m._on_path(_Path(to_l(legs_w[1]), t=1.0))
        _, curve, _ = m._path_in_hand()
        keep = m._legs_left(curve)
        print("   standing %5.3f m off corner 2: %d detected, %d to fly -> %s"
              % (off, len(curve), len(keep),
                 "the shared corner is dropped" if len(keep) == 1 else "it is treated as a leg to fly"))
        if len(keep) != want:
            problems.append("standing %.3f m off the shared corner gave %d corners to fly, expected %d"
                            % (off, len(keep), want))

    # ---- 3. the closed tour: the shape RECT1/RECT6 were flown with ------------------------------------
    m = make_mission(calib, np.zeros(3))
    m._on_path(_Path(to_l(densify(c, args.spacing)), t=99.0))
    path_f, curve, closed = m._path_in_hand()
    keep = m._legs_left(curve)
    print()
    print("the closed tour (hold OFF, RECT1/RECT6's shape): %d points -> %d corners, closed=%s, "
          "%d to fly from the pad" % (len(path_f), len(curve), closed, len(keep)))
    if not closed:
        problems.append("a closed tour was not classified as closed")
    if len(curve) != n_c + 1 or len(keep) != n_c + 1:
        problems.append("a closed tour should be %d corners (%d + the closing point) and none of them is "
                        "behind the vehicle on the pad; got %d detected / %d to fly"
                        % (n_c + 1, n_c, len(curve), len(keep)))

    # ---- 4. the EMPTY path ----------------------------------------------------------------------------
    m.traj_last_empty = False
    m._on_path(_Path(np.zeros((0, 3)), t=100.0))
    print()
    print("an EMPTY Path (the source saying 'no more legs'): traj_last_empty=%s" % m.traj_last_empty)
    if not m.traj_last_empty:
        problems.append("an EMPTY path did not set traj_last_empty, so a tour could only end by timeout")

    print()
    if problems:
        print("CORNER-TOUR TEST FAILED (%d problems):" % len(problems))
        for p in problems:
            print("   - %s" % p)
        return 1
    print("CORNER-TOUR TEST PASSED: %d leg phases for the %d-corner square, the shared corner dropped, the "
          "closed tour unchanged, the empty path honoured." % (len(flown), n_c))
    return 0


if __name__ == "__main__":
    sys.exit(main())
