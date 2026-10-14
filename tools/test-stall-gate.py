#!/usr/bin/env python3
"""The F3 stall gate, tested WITHOUT a flight and without a ROS graph.

WHY A TEST AND NOT A SORTIE: the gate decides whether a healthy mission gets aborted, and both directions of
that decision are cheap to check here and expensive to discover in the air.  `Mission._stall_reason` is pure
logic over five attributes, so it can be called on a stub object.

The three cases are exactly the three states a sortie can be in:
  A. the planner says "stalled" but plans are still arriving (it is waiting for the aircraft) -> NO abort;
  B. the planner says "stalled" and nothing has arrived for --stall-abort-s                    -> ABORT, named;
  C. the signal cleared (a new plan was published)                                             -> NO abort.

usage: python3 tools/test-stall-gate.py
"""
from __future__ import annotations

import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)


def main() -> int:
    # import the class without starting anything (mission_fsm imports rclpy only inside start_ros)
    import importlib.util
    spec = importlib.util.spec_from_file_location("mission_fsm_t", os.path.join(ROOT, "fsm", "mission_fsm.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    ok = True

    def check(name, good, detail=""):
        nonlocal ok
        ok = ok and bool(good)
        print("%-70s %s %s" % (name, "ok" if good else "FAIL", detail))

    def stub(stalled, up_s, quiet_s):
        s = object.__new__(mod.Mission)
        s.args = argparse.Namespace(stall_abort_s=3.0, stall_topic="/planner/stalled",
                                   traj_topic="/planner/traj")
        s.planner_stalled = stalled
        now = time.monotonic()
        s.planner_stall_since = None if not stalled else now - up_s
        s.last_path_wall = now - quiet_s
        return s

    r = mod.Mission._stall_reason(stub(True, 10.0, 0.2))
    check("A. stalled signal up 10 s but a plan arrived 0.2 s ago -> NO abort", r is None, str(r)[:40])

    r = mod.Mission._stall_reason(stub(True, 10.0, 10.0))
    check("B. stalled signal up 10 s and no plan for 10 s -> ABORT, and it NAMES the planner",
          r is not None and "THE PLANNER IS NOT PLANNING" in r and "NOT a trajectory-source timeout" in r,
          (r or "")[:70])

    r = mod.Mission._stall_reason(stub(True, 1.0, 1.0))
    check("C. stalled, but for less than --stall-abort-s -> NO abort (a 2 s hiccup is not a death)",
          r is None, str(r)[:40])

    r = mod.Mission._stall_reason(stub(False, 0.0, 30.0))
    check("D. signal down (a new plan was published) -> NO abort even after a long silence", r is None,
          str(r)[:40])

    print("stall-gate test:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
