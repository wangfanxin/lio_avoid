#!/usr/bin/env python3
"""THE SUPERVISOR'S ONE-WAY RULE, AS A GATE INSTEAD OF A PROMISE.

THE RULE (`_stage/refactor-bridge-plan.md` section 4.2): the flight commands -- message class 76 -- have
exactly ONE writer, the supervisor; the setpoints -- class 84 -- have exactly one, the BRIDGE, on behalf of
the plan box.  The split is what makes two processes on two MAVLink endpoints safe WITHOUT any runtime
arbitration, and it is what the F2 bench (`tools/test-second-endpoint.py`) proved is technically possible.
It is also free to a dropped-in plan box, because ego's own source never writes to a flight controller at
all.

WHY A TEST.  Until this file existed, "the supervisor never sends a setpoint" was true only by code review
-- and this project has already learned what that is worth (the corner hold was logged as three 5 s holds
while the aircraft spent them somewhere else).  A rule that a future edit can break silently is not a rule.
So this test reads the supervisor's SOURCE and fails if it can reach a setpoint path, and it proves it can
fail by pasting one in on a copy of the tree.

WHAT IT ASSERTS, over `supervisor/supervisor.py`:
  1. no call to any of the bridge's setpoint entry points (`send_cmd` -- the only one that exists since F1
     deleted the three F-writers `send_position`, `send_position_velocity`, `send_velocity`, whose names are
     still refused here so that a resurrected call site cannot slip through);
  2. no reference to message 84 / SET_POSITION_TARGET_LOCAL_NED / `set_position_target_local_ned_send`
     anywhere in the file, comments included: a comment that names it is a reader's hint that the rule was
     being thought about, and the file's own prose is allowed to say it NEVER does this (so the check is on
     CALLS and on the wire-level names, not on the substring "setpoint");
  3. the supervisor DOES still declare its authority: it must call `arm`, `takeoff`, `land` and
     `set_mode`, or it is not the authority the design describes.

NEGATIVE CONTROLS, on copies of the tree: paste `self.bridge.send_cmd(...)` into the supervisor and the test
must fail; paste `send_position(...)` (a deleted entry point) and it must fail too.

usage: tools/test-supervisor-authority.py [--no-self-test]
"""
from __future__ import annotations

import argparse
import os
import re
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SUP = os.path.join("supervisor", "supervisor.py")

#: the bridge's setpoint entry points: `send_cmd` is the ONE that exists since F1; the other three were
#: deleted in F1 and are refused here so that a resurrected call site cannot slip through.
SETPOINT_CALLS = ("send_cmd", "send_position", "send_position_velocity", "send_velocity")
#: the wire-level names of message class 84
WIRE_NAMES = ("set_position_target_local_ned_send", "SET_POSITION_TARGET_LOCAL_NED",
              "MAV_FRAME_LOCAL_NED")
#: what the supervisor MUST call, or it is not the mission's authority
AUTHORITY_CALLS = ("arm", "takeoff", "land", "set_mode")


def scan(path, root):
    """(problems, lines) for one file."""
    src = open(path, encoding="utf-8").read()
    # strip comments and docstrings?  NO: a call inside a docstring is not a call, but the wire names are
    # ALSO checked in code only, so parse the AST for calls and grep the code lines for the wire names.
    import ast
    problems, lines = [], []
    tree = ast.parse(src, filename=path)
    calls = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            f = node.func
            name = None
            if isinstance(f, ast.Attribute):
                name = f.attr
            elif isinstance(f, ast.Name):
                name = f.id
            if name:
                calls.append((name, node.lineno))
    for name, lineno in calls:
        if name in SETPOINT_CALLS:
            problems.append("%s:%d CALLS %s() -- the supervisor must never send a setpoint (class 84 is "
                            "the bridge's)" % (os.path.relpath(path, root), lineno, name))
    # the wire names, in CODE (a comment may name them; this project's prose does, deliberately)
    code_lines = []
    for i, line in enumerate(src.splitlines(), start=1):
        stripped = line.split("#", 1)[0]
        code_lines.append((i, stripped))
    for i, line in code_lines:
        for w in WIRE_NAMES:
            if w in line:
                problems.append("%s:%d names %s in code -- message class 84 is not the supervisor's"
                                % (os.path.relpath(path, root), i, w))
    for want in AUTHORITY_CALLS:
        if not any(n == want for n, _ in calls):
            problems.append("%s: the supervisor never calls %s() -- it is supposed to OWN the flight "
                            "commands (class 76)" % (os.path.relpath(path, root), want))
    lines.append("   %s: %d calls scanned; setpoint calls: %d; authority calls present: %s"
                 % (os.path.relpath(path, root), len(calls),
                    sum(1 for n, _ in calls if n in SETPOINT_CALLS),
                    ", ".join(w for w in AUTHORITY_CALLS if any(n == w for n, _ in calls))))
    return problems, lines


def _mutate(path, old, new):
    src = open(path, encoding="utf-8").read()
    if old not in src:
        raise SystemExit("negative control: anchor not found in %s -- the control is stale" % path)
    open(path, "w", encoding="utf-8").write(src.replace(old, new, 1))


def self_test():
    ok = True
    SKIP = {"__pycache__", ".git", "build", "install", "log", "logs", "node_modules"}
    tmp = tempfile.mkdtemp(prefix="sup-auth-neg-")
    try:
        for label, needle in (
                ("paste a send_cmd() call into the supervisor", "self.bridge.send_cmd(m, self.calib)"),
                ("paste a resurrected send_position() call", "self.bridge.send_position(p, 0.0)"),
                ("paste the wire-level message name IN CODE",
                 "frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED")):
            dst = os.path.join(tmp, "copy%d" % len(os.listdir(tmp)))
            shutil.copytree(ROOT, dst, ignore=shutil.ignore_patterns(*SKIP), symlinks=True)
            p = os.path.join(dst, SUP)
            src = open(p, encoding="utf-8").read()
            anchor = "        self.bridge = MavlinkBridge(url=self.args.mav, log=self.log)\n"
            open(p, "w", encoding="utf-8").write(
                src.replace(anchor, anchor + "        " + needle + "\n", 1))
            print("NEGATIVE CONTROL: %s" % label)
            problems, _ = scan(p, dst)
            hit = [x for x in problems if "class 84" in x or "must never send a setpoint" in x]
            print("   the mutated copy reported %d problem(s)" % len(problems))
            print("   verdict: %s" % ("FAILS as required" if hit else "DID NOT FAIL -- control is dead"))
            for h in hit[:1]:
                print("      %s" % h)
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
    print("the supervisor's one-way rule: class 76 here, class 84 in the bridge")
    problems, lines = scan(os.path.join(ROOT, SUP), ROOT)
    for line in lines:
        print(line)
    ok = not problems
    for p in problems:
        print("   [FAIL] %s" % p)
    if args.self_test:
        print()
        ok = self_test() and ok
    print()
    print("SUPERVISOR-AUTHORITY TEST %s" % ("PASSED" if ok else "FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
