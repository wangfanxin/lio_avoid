#!/usr/bin/env python3
"""S4: the L -> W -> wire conversion moved into the bridge, proven BIT-IDENTICAL to the old chain.

WHAT MOVED.  The chain that commands a setpoint used to convert the plan's point in two places:

    old   fsm/mission_fsm.py:446-449   p_f = R_WL @ p_L + t_WL        (the mission's L -> "F" step)
          bridge/mavlink_impl.py:345   (N, E, -U) of p_f              (the transport's F -> NED step)

    new   bridge/mavlink_impl.py       send_cmd(cmd_l, calib): the SAME expression, in ONE place,
                                       called by the transport instead of by the mission.

`F` is `W` numerically, because the EKF origin is set to the pad.  That identity is what made the split
survive, and it is exactly what a boundary refactor must stop depending on silently -- so the move has to
be provable, and because it is pure arithmetic it can be proven with ZERO tolerance.

WHAT IS COMPARED, and why zero tolerance is the right number.  Both paths are driven with the SAME L
point and the SAME `common.frames.Calib`, in the SAME process, and the comparison is on the ARGUMENTS
HANDED TO PYMAVLINK -- the six numbers, the frame id, the type mask and the yaw that `send_position` and
`send_cmd` each pass to `set_position_target_local_ned_send`.  Not "close": `repr()`-equal, i.e.
bit-identical, and the test fails on the first differing byte.  The old path applies `R_WL @ p_L + t_WL`
and then a permutation; the new path calls `frames.l_to_wire` (the same expression, one call) and the same
permutation.  Composing the rotation, the offset and the permutation into ONE 3x4 matrix would have been
mathematically equal and NOT bit-identical, because the sum accumulates in a different order -- which is
why `frames.l_to_wire` deliberately keeps them separate (`common/frames.py`, its docstring).

THE TABLE: several poses (including the four mission corners and values with no exact binary form), three
velocities and accelerations, five yaws, and THREE calibrations -- the measured one the flight used, the
prediction, and a deliberately composed one (a rotation and a translation that are both irrational in
binary).  A single case could pass by luck; the comparison is over every case at once.

THE NEGATIVE CONTROL is run on a COPY of this repository in a temporary directory (so the tree under test
is never mutated): `R_WL` is replaced by its transpose in `bridge/mavlink_impl.py`'s conversion, and the
test must then FAIL.  The mutation is the one the old `planner/ego_adapter.py` self-test used
(refactor-bridge-plan.md section 6, step S4).  A second control removes `t_WL` from the position and must
also fail -- a transpose alone can hide behind a rotation that happens to be nearly square.

usage: tools/test-s4-l-to-wire.py [--no-self-test]
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
import shutil
import sys
import tempfile
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from common import frames                            # noqa: E402

POSES_L = [
    [0.0, 0.0, 0.0],
    [0.5, 0.4, 2.0],
    [2.5, 0.4, 2.0],
    [2.5, 1.6, 2.0],
    [0.5, 1.6, 2.0],
    [1.0, 2.0, 3.0],
    [-3.14159265358979, 2.718281828459045, 1.4142135623730951],
    [0.1, 0.2, 0.3],
    [1e-9, -1e-9, 5e-10],
    [123.456789, -987.654321, 0.000000123],
]
VELS_L = [[0.0, 0.0, 0.0], [0.3, 0.0, 0.0], [0.1, -0.2, 0.05],
          [1.0 / 3.0, 2.0 / 7.0, -1.0 / 9.0], [0.0, 0.0, -0.123456789]]
YAW_DEG = [0.0, 30.0, 90.0, -179.9, 123.456]


class Recorder:
    """Stands in for pymavlink's MAVLink object: records the calls."""

    def __init__(self):
        self.calls = []

    def __getattr__(self, name):
        def rec(*args, **kwargs):
            self.calls.append((name, args, kwargs))
        return rec


class FakeConn:
    def __init__(self):
        self.mav = Recorder()
        self.target_system = 1
        self.target_component = 1


class Cmd:
    """Anything with p / v / a / yaw -- the shape `send_cmd` documents."""

    def __init__(self, p, v, a, yaw):
        self.p = np.asarray(p, dtype=float)
        self.v = np.asarray(v, dtype=float)
        self.a = np.asarray(a, dtype=float)
        self.yaw = float(yaw)


def make_bridge(module):
    """A MavlinkBridge with no socket (the module's own class, from whichever tree is under test)."""
    mi = module
    br = mi.MavlinkBridge.__new__(mi.MavlinkBridge)
    br.mav = FakeConn()
    br.log = lambda s: None
    br._sent_positions = 0
    br._sent_nav = 0
    return br


def setpoint_call(br):
    """The last `set_position_target_local_ned_send` arguments, verbatim."""
    for name, args, kwargs in reversed(br.mav.mav.calls):
        if name == "set_position_target_local_ned_send":
            return args
    return None


def load_module(root, name):
    """Import `bridge.mavlink_impl` FROM `root`, with the modules it reaches purged first.

    THE PURGE IS THE WHOLE CONTROL, and it has to cover `common` too.  `frames.l_to_wire` is called from
    inside `send_cmd` via `from common import frames`, so a cached `common.frames` would serve the
    ORIGINAL tree's function to a mutated copy -- which is exactly what happened: the first version of
    this test purged only `bridge.*`, and both `frames.py` mutations reported "0 problems" on a copy whose
    `frames.py` had been transposed.  A negative control that cannot see its own mutation is not a
    control, so the purge is by PREFIX.
    """
    import importlib
    for mod in [m for m in sys.modules
                if m == "common" or m.startswith("common.") or m == "bridge" or m.startswith("bridge.")]:
        sys.modules.pop(mod, None)
    # the tree under test must be FIRST on the path, and every other candidate removed
    sys.path[:] = [p for p in sys.path if p not in (root, ROOT, os.path.dirname(root))]
    sys.path.insert(0, root)
    spec = importlib.util.spec_from_file_location(
        "%s_under_test" % name, os.path.join(root, "bridge", "mavlink_impl.py"))
    mod = importlib.util.module_from_spec(spec)
    mod.__package__ = "bridge"          # so its `from bridge.base import ...` resolves against `root`
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    # sanity: the module must be the copy's, and so must the `common` it reaches
    assert os.path.realpath(mod.__file__) == os.path.realpath(
        os.path.join(root, "bridge", "mavlink_impl.py")), mod.__file__
    import common.frames as cf
    assert os.path.realpath(cf.__file__) == os.path.realpath(
        os.path.join(root, "common", "frames.py")), cf.__file__
    return mod


def calibrations(fr=None):
    """The calibrations, built with the CALIBRATION CLASS OF THE TREE UNDER TEST (`fr = common.frames`).

    Using the tree's own `Calib` matters for the controls: a mutation of `common/frames.py` has to be
    reachable from everything the test builds, not only from the code path `send_cmd` happens to take.
    """
    fr = fr or frames
    cals = [("predicted (spawn 0 deg, body axes)",
             fr.Calib.predicted(spawn_heading_deg=0.0, world_axes="body")),
            ("predicted (spawn 37 deg, sensor axes)",
             fr.Calib.predicted(spawn_heading_deg=37.0, world_axes="sensor")),
            ("composed: yaw 123 deg, t = (0.31, -0.22, 0.07)",
             fr.Calib(R_WL=fr.R_z(math.radians(123.0)),
                      t_WL=np.array([0.31, -0.22, 0.07])))]
    measured = "/mnt/d/distill_self/_stage/calib.json"
    if os.path.exists(measured):
        d = json.load(open(measured))
        cals.insert(0, ("MEASURED (the calibration the flights used)",
                        fr.Calib(R_WL=np.array(d["R_WL"]), t_WL=np.array(d["t_WL"]))))
    return cals


def compare(module, quiet=False):
    """Returns (problems, n_cases, lines)."""
    problems, lines = [], []
    import common.frames as fr          # the tree-under-test's own frames module (see load_module)
    n = 0
    for cal_name, calib in calibrations(fr):
        for p_l in POSES_L:
            for v_l in VELS_L:
                for a_l in VELS_L:
                    for yaw_deg in YAW_DEG:
                        yaw = math.radians(yaw_deg)
                        # ---- OLD PATH: the mission converts L -> W, the transport permutes W -> NED
                        p_f = calib.R_WL @ np.asarray(p_l, dtype=float) + calib.t_WL
                        v_f = calib.R_WL @ np.asarray(v_l, dtype=float)
                        a_f = calib.R_WL @ np.asarray(a_l, dtype=float)
                        br_old = make_bridge(module)
                        br_old.send_position_velocity(p_f, v_f, yaw, a_f)
                        old = setpoint_call(br_old)
                        # ---- NEW PATH: the transport does both, in one place
                        br_new = make_bridge(module)
                        cmd = Cmd(p_l, v_l, a_l, yaw)
                        br_new.send_cmd(cmd, calib)
                        new = setpoint_call(br_new)
                        n += 1
                        where = "%s p=%s v=%s a=%s yaw=%.3f" % (cal_name, p_l, v_l, a_l, yaw)
                        if old is None or new is None:
                            problems.append("%s: one of the two paths sent nothing" % where)
                            continue
                        if len(old) != len(new):
                            problems.append("%s: %d vs %d arguments" % (where, len(old), len(new)))
                            continue
                        for i, (x, y) in enumerate(zip(old, new)):
                            if repr(x) != repr(y):
                                problems.append("%s: argument %d differs -- old %r, new %r"
                                                % (where, i, x, y))
        lines.append("   %-46s %d cases compared" % (cal_name, n))
    return problems, n, lines


def check_names(module, quiet=False):
    """The two arguments a consumer reads by NAME rather than by value must also be equal."""
    problems = []
    import common.frames as fr
    calib = calibrations(fr)[0][1]
    p_l = [1.0, 2.0, 3.0]
    yaw = math.radians(30.0)
    p_f = calib.R_WL @ np.asarray(p_l, dtype=float) + calib.t_WL
    br_old = make_bridge(module)
    br_old.send_position_velocity(p_f, np.zeros(3), yaw, np.zeros(3))
    old = setpoint_call(br_old)
    br_new = make_bridge(module)
    br_new.send_cmd(Cmd(p_l, [0, 0, 0], [0, 0, 0], yaw), calib)
    new = setpoint_call(br_new)
    for i, label in ((3, "MAV_FRAME_ID"), (4, "type mask"), (14, "yaw"), (15, "yaw_rate")):
        if repr(old[i]) != repr(new[i]):
            problems.append("%s differs: old %r, new %r" % (label, old[i], new[i]))
    return problems


def _mutate(path, old, new):
    src = open(path, encoding="utf-8").read()
    if old not in src:
        raise SystemExit("negative control: %r not found in %s -- the control is stale" % (old[:60], path))
    open(path, "w", encoding="utf-8").write(src.replace(old, new, 1))


def self_test(quiet=False):
    """The negative controls, each on its own COPY of this repository."""
    ok = True
    SKIP = {"__pycache__", ".git", "build", "install", "log", "logs", "node_modules"}
    tmp = tempfile.mkdtemp(prefix="s4-neg-")
    try:
        cases = [
            ("transpose R_WL in the bridge's conversion",
             lambda r: _mutate(os.path.join(r, "common", "frames.py"),
                               "    return np.asarray(R, dtype=float) @ np.asarray(l, dtype=float)"
                               " + np.asarray(t, dtype=float)",
                               "    return np.asarray(R, dtype=float).T @ np.asarray(l, dtype=float)"
                               " + np.asarray(t, dtype=float)"),
             "differs"),
            ("drop t_WL from the position conversion",
             lambda r: _mutate(os.path.join(r, "common", "frames.py"),
                               "    return np.asarray(R, dtype=float) @ np.asarray(l, dtype=float)"
                               " + np.asarray(t, dtype=float)",
                               "    return np.asarray(R, dtype=float) @ np.asarray(l, dtype=float)"),
             "differs"),
            ("drop the sign flip of z on the way to NED",
             lambda r: _mutate(os.path.join(r, "bridge", "mavlink_impl.py"),
                               "        x_ned, y_ned, z_ned = float(p_f[1]), float(p_f[0]),"
                               " -float(p_f[2])\n        vx, vy, vz = float(v_f[1]), float(v_f[0]),"
                               " -float(v_f[2])\n        ax, ay, az = float(a_f[1]), float(a_f[0]),"
                               " -float(a_f[2])",
                               "        x_ned, y_ned, z_ned = float(p_f[1]), float(p_f[0]),"
                               " float(p_f[2])\n        vx, vy, vz = float(v_f[1]), float(v_f[0]),"
                               " float(v_f[2])\n        ax, ay, az = float(a_f[1]), float(a_f[0]),"
                               " float(a_f[2])"),
             "differs"),
        ]
        for label, mutate, needle in cases:
            dst = os.path.join(tmp, "copy%d" % len(os.listdir(tmp)))
            shutil.copytree(ROOT, dst, ignore=shutil.ignore_patterns(*SKIP), symlinks=True)
            mutate(dst)
            print("NEGATIVE CONTROL: %s" % label)
            mod = load_module(dst, "neg")
            problems, n, _ = compare(mod, quiet=True)
            hit = [p for p in problems if needle in p] or problems
            print("   the mutated copy was compared over %d cases and reported %d problem(s)"
                  % (n, len(problems)))
            print("   verdict: %s" % ("FAILS as required" if hit else "DID NOT FAIL -- control is dead"))
            for p in hit[:2]:
                print("      %s" % p)
            if not hit:
                ok = False
            shutil.rmtree(dst, ignore_errors=True)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    return ok


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--self-test", dest="self_test", action="store_true", default=True)
    ap.add_argument("--no-self-test", dest="self_test", action="store_false")
    args = ap.parse_args()

    print("S4: L -> W -> wire, moved into the bridge, compared at atol = 0.0")
    print("the tree under test: %s" % ROOT)
    print()
    module = load_module(ROOT, "main")
    problems, n, lines = compare(module)
    for l in lines:
        print(l)
    print("   %d cases compared, all with repr() equality on every argument to pymavlink" % n)
    problems += check_names(module)
    ok = not problems
    if problems:
        print()
        print("   FAILURES (%d):" % len(problems))
        for p in problems[:6]:
            print("      %s" % p)
    else:
        print("   IDENTICAL: the relocated expression hands pymavlink the same bytes as the old chain")

    if args.self_test:
        print()
        ok = self_test() and ok

    print()
    print("S4 L->WIRE A/B %s" % ("PASSED" if ok else "FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
