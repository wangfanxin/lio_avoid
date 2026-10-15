#!/usr/bin/env python3
"""PUT `planner/ego_on_lio.launch.py` INTO THE EGO CHECKOUT, BUILD IT, AND CHECK IT AS FAR AS OFFLINE GOES.

WHY A TOOL RATHER THAN A COPY-PASTE.  The launch file that wires ego onto this stack belongs to THIS
repository -- it encodes our topic names, our map box and our ceiling -- but it has to LIVE in the ego
checkout for `ros2 launch ego_planner ego_on_lio.launch.py` to find it, and launch files there are
INDIVIDUALLY SYMLINKED into `planner_ws/install`, so a new one is invisible until the package is rebuilt.
Three steps, and skipping the last one is silent:

    1. copy   src/planner/ego_on_lio.launch.py  ->  <ego>/src/planner/plan_manage/launch/
    2. build  colcon build --symlink-install --packages-select ego_planner   (in planner_ws)
    3. check  --static: python syntax, the LaunchDescription itself, and EVERY parameter name against the
              list the NODE declares.  A launch file that sets a parameter the node never declares does
              nothing at all, and says nothing: the sortie simply flies a default.  This check found four
              real omissions in the first version (`optimization/max_vel`, `optimization/max_acc` and two
              of the four `lambda_*` weights that upstream sets).

`--static` needs no flight and no SITL.  `--run` additionally STARTS the node from the launch file for a
few seconds and asks IT for its declared parameters, which is the only way to check the names against the
real node rather than against a copy of its source; that is what `docs/metrics/ego-launch-20261016.txt`
records.  The live check needs ROS sourced, so this tool does it through a shell rather than in-process.

usage: tools/install-ego-launch.py [--ego DIR] [--ws DIR] [--static] [--run] [--dry-run]
"""
from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CODE = os.path.abspath(os.path.join(HERE, ".."))            # <repo>/src
PROJ = os.path.abspath(os.path.join(CODE, ".."))            # <repo>

SRC_NAME = "ego_on_lio.launch.py"
SRC = os.path.join(CODE, "planner", SRC_NAME)
DEFAULT_EGO = "/home/fanxin/lio_avoid/src/ego-planner-swarm"
DEFAULT_WS = "/home/fanxin/lio_avoid/planner_ws"
REL = "src/planner/plan_manage/launch"                      # inside the ego checkout


def params_in(text: str) -> set:
    """Every `'name':` in the file -- not just the first on each line (the dict puts several on one line)."""
    return set(re.findall(r"'([a-z_]+/[a-z_0-9]+)'\s*:", text))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--ego", default=DEFAULT_EGO, help="the ego-planner-swarm checkout")
    ap.add_argument("--ws", default=DEFAULT_WS, help="the colcon workspace that builds ego_planner")
    ap.add_argument("--static", action="store_true", help="syntax + launch description + parameter names")
    ap.add_argument("--run", action="store_true", help="ALSO start the node and ask it for its parameters")
    ap.add_argument("--dry-run", action="store_true", help="say what would happen, change nothing")
    args = ap.parse_args()
    if not (args.static or args.run):
        args.static = True

    bad = []
    if not os.path.isfile(SRC):
        print("FAILED: %s does not exist" % SRC)
        return 1
    text = open(SRC, encoding="utf-8").read()
    launch_params = params_in(text)
    dst_dir = os.path.join(args.ego, REL)
    dst = os.path.join(dst_dir, SRC_NAME)

    print("source : %s" % os.path.relpath(SRC, PROJ))
    print("target : %s" % dst)
    print("it sets %d parameter(s), in the `name:` form the node reads" % len(launch_params))
    if not os.path.isdir(dst_dir):
        print("FAILED: the ego launch directory does not exist: %s" % dst_dir)
        return 1
    if args.dry_run:
        print("--dry-run: nothing copied, nothing built")
        return 0

    # ------------------------------------------------------------------ 1. copy
    shutil.copyfile(SRC, dst)
    same = open(dst, encoding="utf-8").read() == text
    print("copied, identical: %s" % same)
    if not same:
        bad.append("the copy differs from the source")

    # ------------------------------------------------------------------ 2. build
    if not os.path.isdir(args.ws):
        bad.append("the workspace %s does not exist, so the launch file cannot be installed" % args.ws)
    else:
        r = subprocess.run(["colcon", "build", "--symlink-install", "--packages-select", "ego_planner"],
                           cwd=args.ws, capture_output=True, text=True, timeout=900)
        print("colcon rc=%d  %s" % (r.returncode, (r.stdout.strip().splitlines() or [""])[-1]))
        if r.returncode != 0:
            bad.append("colcon failed; the launch file is NOT installed")
        inst = os.path.join(args.ws, "install", "ego_planner", "share", "ego_planner", "launch", SRC_NAME)
        print("installed: %s  (%s)" % (inst, "present" if os.path.exists(inst) else "*** MISSING ***"))
        if not os.path.exists(inst):
            bad.append("the launch file is not in install/ after the build")

    if args.static:
        r = subprocess.run([sys.executable, "-m", "py_compile", dst], capture_output=True, text=True)
        print("py_compile rc=%d" % r.returncode)
        if r.returncode != 0:
            bad.append("py_compile failed: %s" % r.stderr.strip()[-200:])

    # ------------------------------------------------------------------ 3. check
    if args.run or args.static:
        script = os.path.join(PROJ, "..", "_stage", "compare-ego-launch-params.py")
        cap = os.path.join(PROJ, "..", "_stage", "ego-launch-params.txt")
        if args.run:
            sh = os.path.join(PROJ, "..", "_stage", "check-ego-launch-params.sh")
            if os.path.isfile(sh):
                subprocess.run(["bash", sh], capture_output=True, text=True, timeout=900)
        if os.path.isfile(cap) and os.path.isfile(script):
            r = subprocess.run([sys.executable, script], capture_output=True, text=True, timeout=300)
            declared = set(re.findall(r"^\s+([a-z_]+/[a-z_0-9]+)\s*$",
                                      open(cap, encoding="utf-8", errors="replace").read(), re.M))
            missing = sorted(launch_params - declared) if declared else []
            print("the node declares %d parameter(s); set by us but NOT declared: %d"
                  % (len(declared), len(missing)))
            for m in missing:
                print("   *** %s" % m)
            if missing:
                bad.append("%d parameter name(s) the node does not declare -- each would do nothing" % len(missing))
            if r.returncode != 0 and not missing:
                bad.append("the comparison tool exited %d" % r.returncode)
        else:
            print("NOTE  no captured node parameter list at %s; run with --run to produce one" % cap)

    print()
    if bad:
        print("EGO LAUNCH INSTALL FAILED (%d problem(s)):" % len(bad))
        for b in bad:
            print("   - %s" % b)
        return 1
    print("EGO LAUNCH INSTALL PASSED: copied, built into the workspace, syntax-checked, and every parameter "
          "name is one the node declares")
    return 0


if __name__ == "__main__":
    sys.exit(main())
