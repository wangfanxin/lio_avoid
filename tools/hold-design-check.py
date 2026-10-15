#!/usr/bin/env python3
"""Can the corner hover be delivered by the consumer UNCHANGED?  An answer from the consumer's own code.

THE QUESTION (2026-10-15).  The owner's requirement is a 5 s hover at each of the four corners, and the
mechanism has to be chosen before it is flown.  Two designs were on the table:

  (i)  the producer publishes the FULL remaining tour as ONE Path, with the dwell expressed as REPEATED
       POSES at each corner, so `fsm/mission_fsm.py --follow corner` works unchanged;
  (ii) the producer publishes ONE LEG PER MESSAGE, advancing ON ARRIVAL, and the FSM keeps walking its
       corner list and accepts a new Path mid-mission.

This tool decides between them with the pieces that would actually run: `_corners_of` (the FSM's own corner
detector, imported from the FSM, not copied), `GATES["settle_s"]` (the only hold the FSM's corner walk
contains), and the two path shapes the producer can publish.  Nothing here is a simulation of the FSM -- it
is the FSM's detector and the FSM's gate, applied to the two encodings.

WHAT IT ASSERTS (it exits 1 if any of it stops being true):
  1. the closed tour `densify()` produces (the shape RECT1/RECT6 flew) is detected as 5 corners: the four
     mission corners plus the CLOSING POINT, which is why a closed tour is flown as 5 legs, not 4;
  2. one OPEN leg (what design (ii) publishes per message) is detected as exactly its two ENDS -- an open
     polyline is wrapped by the detector, so its ends read as 180 deg turns.  That is what makes a leg
     usable as "one leg" and why the consumer drops the end it is already standing on;
  3. repeated poses at a corner do NOT encode a dwell: they never produce more than two detected corners
     at that vertex, and for K >= 4 the vertex is not detected AT ALL -- the encoding does not just fail to
     hold the corner, it deletes it, and the consumer then flies a straight line past it;
  4. the consumer's corner walk contains exactly one hold -- the settle loop -- bounded by
     `GATES["settle_s"]`, measured on RECT6 at 0.1-0.9 s per corner.  Any value below the owner's 5 s
     cannot deliver the requirement, whatever the path encoding does.

usage: tools/hold-design-check.py [--world worlds/pillars.yaml] [--spacing 0.05] [--hold-s 5.0]
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from fsm.mission_fsm import GATES, _corners_of        # noqa: E402  the code under test, not a copy
from planner.rectangle_source import densify          # noqa: E402


def read_world(path):
    """`tools/world-info.py` by file path: the file name has a dash, so it is not importable by name."""
    import importlib.util
    spec = importlib.util.spec_from_file_location("world_info_holddesign",
                                                  os.path.join(HERE, "world-info.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.read(path)


def corners_at(path, vertex, tol=1e-6):
    """How many detected corners sit on `vertex`."""
    return sum(1 for c in path if float(np.linalg.norm(np.asarray(c[:2]) - np.asarray(vertex[:2]))) <= tol)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default=os.path.join(ROOT, "worlds", "pillars.yaml"))
    ap.add_argument("--spacing", type=float, default=0.05)
    ap.add_argument("--hold-s", type=float, default=5.0, help="the dwell the owner asked for, s")
    args = ap.parse_args()

    world = read_world(args.world)
    corners = np.asarray([[c[0], c[1], c[2]] for c in world["corners"]], dtype=float)
    # the world file lists the closing point explicitly; the producer drops it and closes the loop itself
    if len(corners) > 1 and np.allclose(corners[0], corners[-1]):
        corners = corners[:-1]
    print("world %s: %d mission corners, hold asked %.1f s, GATES['settle_s'] = %.1f s"
          % (os.path.basename(args.world), len(corners), args.hold_s, GATES["settle_s"]))
    problems = []

    # ---- 1. the CLOSED tour: one Path, all four legs (RECT1/RECT6's shape, and design (i)'s shape) -----
    closed = densify(corners, args.spacing)
    c_closed = _corners_of(closed)
    print()
    print("encoding A -- ONE CLOSED TOUR (%d points, %.3f m):" % (len(closed), float(
        np.sum(np.linalg.norm(np.diff(closed[:, :2], axis=0), axis=1)))))
    print("   _corners_of -> %d corners" % len(c_closed))
    for i, c in enumerate(c_closed):
        print("      corner %d: E %+7.3f N %+7.3f" % (i + 1, c[0], c[1]))
    if len(c_closed) != len(corners) + 1:
        problems.append("a closed tour of %d corners was detected as %d corners (expected %d: the four "
                        "mission corners plus the closing point)"
                        % (len(corners), len(c_closed), len(corners) + 1))
    if len(c_closed) >= 2 and float(np.linalg.norm(c_closed[0][:2] - c_closed[-1][:2])) > 1e-6:
        problems.append("the closed tour's first and last detected corners are not the same point")
    print("   -> %d leg phases: the four mission corners plus the CLOSING POINT (measured: RECT6's B2 "
          "table has 5 rows, 0.594 / 2.111 / 1.200 / 2.071 / 1.166 m)" % len(c_closed))

    # ---- 2. ONE OPEN LEG: what design (ii) publishes per message ---------------------------------------
    a, b = corners[0], corners[1]
    n = max(2, int(float(np.linalg.norm(b - a)) / max(args.spacing, 1e-3)) + 1)
    leg = np.linspace(a, b, n)
    c_leg = _corners_of(leg)
    print()
    print("encoding B -- ONE OPEN LEG (design (ii), %d points, %.3f m):" % (len(leg),
                                                                          float(np.linalg.norm(b - a))))
    print("   _corners_of -> %d corners: %s" % (len(c_leg), " ".join("(E %+.3f N %+.3f)" % (c[0], c[1])
                                                                    for c in c_leg)))
    if len(c_leg) != 2:
        problems.append("an open leg was detected as %d corners, not its two ends" % len(c_leg))
    else:
        if float(np.linalg.norm(c_leg[0][:2] - a[:2])) > 1e-6 or \
                float(np.linalg.norm(c_leg[1][:2] - b[:2])) > 1e-6:
            problems.append("an open leg's two detected corners are not its own endpoints")
        else:
            print("   -> the leg's two ENDS, in order.  The consumer drops the end it is already parked on, "
                  "so each message is one leg phase.")

    # ---- 3. design (i)'s dwell encoding: REPEATED POSES at a corner ------------------------------------
    print()
    print("encoding C -- design (i): the CLOSED TOUR with K repeated poses at corner 2 (E %+.3f N %+.3f):"
          % (corners[1][0], corners[1][1]))
    print("   %5s %10s %14s %s" % ("K", "corners", "at corner 2", "the vertex is"))
    for k in (2, 3, 4, 8, 16, 100):
        idx = int(np.argmin(np.linalg.norm(closed[:, :2] - corners[1][:2], axis=1)))
        dup = np.repeat(closed[idx:idx + 1], k - 1, axis=0)
        path = np.vstack([closed[:idx + 1], dup, closed[idx + 1:]])
        det = _corners_of(path)
        at = corners_at(det, corners[1])
        state = "DETECTED (max 2 trivial visits, ~0 s each)" if at else "MISSING -- the turn is gone"
        print("   %5d %10d %14d %s" % (k, len(det), at, state))
        if at > 2:
            problems.append("K=%d produced %d detected corners at one vertex -- the assumption that "
                            "repetition cannot encode a hold is wrong" % (k, at))
        if k >= 4 and at != 0:
            problems.append("K=%d still detected the vertex (%d corners); the measured behaviour is that "
                            "K >= 4 loses it" % (k, at))

    # ---- 4. the only hold the corner consumer has ------------------------------------------------------
    print()
    print("the consumer's own hold: `legs()` holds a corner in the SETTLE loop only -- it exits as soon as "
          "the FCU's horizontal speed is below %.2f m/s, and never later than GATES['settle_s'] = %.1f s."
          % (GATES["settle_speed_mps"], GATES["settle_s"]))
    print("   measured on logs/missionA-RECT6-clean-square/fsm.log: corner 1..5 settled after "
          "0.1 / 0.9 / 0.3 / 0.7 / 0.2 s")
    if GATES["settle_s"] >= args.hold_s:
        problems.append("GATES['settle_s'] is %.1f s, so the unchanged consumer COULD hold %.1f s -- the "
                        "design decision below would have to be re-made"
                        % (GATES["settle_s"], args.hold_s))
    else:
        print("   -> %.1f s < the owner's %.1f s: NO path encoding leaves this consumer able to hover that "
              "long, which is why the dwell cannot live in the consumer's corner walk alone."
              % (GATES["settle_s"], args.hold_s))

    print()
    if problems:
        print("HOLD-DESIGN CHECK FAILED (%d problems):" % len(problems))
        for p in problems:
            print("   - %s" % p)
        return 1
    print("HOLD-DESIGN CHECK PASSED.")
    print("   design (i)  [closed tour + repeated poses, consumer unchanged]: REJECTED -- repetition is not "
          "a dwell (and K >= 4 deletes the corner), and the consumer's only hold is bounded by "
          "GATES['settle_s'] = %.1f s < %.1f s." % (GATES["settle_s"], args.hold_s))
    print("   design (ii) [one leg per message, advance on arrival]: CHOSEN -- each message is a leg the "
          "consumer can fly, the shared corner is dropped by the arrival gate it already enforces, and the "
          "dwell is the interval in which the source has not yet published the next leg, during which the "
          "consumer keeps commanding the corner it is parked on.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
