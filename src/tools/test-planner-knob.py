#!/usr/bin/env python3
"""THE `PLANNER=` KNOB, CHECKED INSTEAD OF TRUSTED — and it is checked because it was BROKEN.

WHAT WENT WRONG, and why this file exists.  The first version of the selector in `tools/run-mission.sh`
stood about forty lines too early: it built the planner's argv while `$VERTICAL_ARG` and `$TRAJ_CALIB_ARG`
did not exist yet.  Under `set -u` that kills the script on the first unbound variable, so
`PLANNER=rectangle` — the DEFAULT, the planner every accepted sortie flies — could not start at all; and had
`set -u` not been there, the plan box would have been launched WITHOUT the vertical climb leg and WITHOUT
the L->W calibration, which is the class of defect that ruins a sortie quietly.

SO THIS TEST RUNS THE REAL SCRIPT.  `PLANNER_DRY_RUN=1` makes `run-mission.sh` print the planner it would
start and the exact argv it would hand it, and exit 0 BEFORE the sortie lock, before ROS and before SITL --
so this test cannot collide with a live flight and cannot leave a lock behind.  For each selection it
asserts:

  1. the script exits 0 (a selector that crashes is not a selector);
  2. the file it names EXISTS and is a file;
  3. `rectangle` (the default) carries --calib-file, --hold-s, the vertical leg when the take-off profile
     asks for one, and the shaping flag when SHAPING_M > 0 -- i.e. the argv is the SAME one the pre-knob
     harness passed, which is what "zero change for the flown planner" has to mean;
  4. `ego` names planner/ego_source.py and does NOT carry the rectangle's flags;
  5. an unknown name is REFUSED with a non-zero exit, and a path that does not exist is REFUSED;
  6. the run leaves no sortie lock behind.

usage: tools/test-planner-knob.py            (exit 0 = the switch selects what it says it selects)
"""
from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CODE = os.path.abspath(os.path.join(HERE, ".."))
PROJ = os.path.abspath(os.path.join(CODE, ".."))
SCRIPT = os.path.join(HERE, "run-mission.sh")
LOCK = "/tmp/lio_avoid_sortie.lock"


def dry_run(env_extra: dict) -> tuple:
    """Run the harness with the dry-run switch.  Returns (rc, stdout+stderr, the argv line)."""
    env = dict(os.environ)
    env.update(env_extra)
    env["PLANNER_DRY_RUN"] = "1"
    # a tag of its own, so a dry run never touches a real sortie's directory
    r = subprocess.run(["bash", SCRIPT, "knob-dry-run"], capture_output=True, text=True, timeout=300,
                       cwd=PROJ, env=env)
    out = r.stdout + r.stderr
    argv = ""
    for line in out.splitlines():
        if line.strip().startswith("argv "):
            argv = line.split(":", 1)[1].strip()
    return r.returncode, out, argv


def field(out: str, name: str) -> str:
    for line in out.splitlines():
        s = line.strip()
        if s.startswith(name + " ") or s.startswith(name + ":"):
            return s.split(":", 1)[1].strip()
    return ""


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.parse_args()

    bad = []
    print("the harness under test: %s" % os.path.relpath(SCRIPT, PROJ))
    print("the switch: PLANNER_DRY_RUN=1 -- prints the selection and the argv, starts nothing\n")

    if os.path.exists(LOCK):
        print("NOTE  a sortie lock exists at %s; the dry runs exit before the lock, so this test does not\n"
              "      touch it, but a real flight may be live -- read the results with that in mind." % LOCK)

    # ---------------------------------------------------------------- 1. the default, and it is the flown one
    rc, out, argv = dry_run({})
    print("PLANNER unset (the default)")
    print("   rc=%d   file=%s" % (rc, field(out, "file")))
    print("   argv: %s" % argv)
    if rc != 0:
        bad.append("the DEFAULT selection exits %d -- the harness cannot start (`set -u` and an undefined "
                   "variable is what this test was written for): %s" % (rc, out.strip().splitlines()[-1:]))
    pf = field(out, "file")
    if not pf.endswith("planner/rectangle_source.py"):
        bad.append("the default does not select planner/rectangle_source.py (got %r)" % pf)
    if not os.path.isfile(pf):
        bad.append("the default names a file that does not exist: %s" % pf)
    # 3. the flown planner's argv must be unchanged by the knob: every flag the pre-knob harness passed
    for flag in ("--world", "--calib-file", "--hold-s"):
        if flag not in argv:
            bad.append("the default's argv is missing %s: %r" % (flag, argv))
    print()

    # 3b. AND THE ENVIRONMENT MUST ACTUALLY REACH THE ARGV, in BOTH directions.  Asserting only that the
    #     climb leg is present cannot tell "the knob decides it" from "the flag is hard-coded" -- so the
    #     leg is asked for and then asked NOT to be there, and both answers are checked.
    rc, out, argv = dry_run({"TAKEOFF_COMMAND_M": "0.2", "VERTICAL_FIRST": "1"})
    print("PLANNER=rectangle, VERTICAL_FIRST=1 (the promoted take-off profile)")
    print("   argv: %s" % argv)
    if rc != 0 or "--vertical-first-m 2.50" not in argv:
        bad.append("VERTICAL_FIRST=1 did not put the climb leg in the argv (rc=%d): %r" % (rc, argv))
    print()
    rc, out, argv = dry_run({"TAKEOFF_COMMAND_M": "0.2", "VERTICAL_FIRST": "0"})
    print("PLANNER=rectangle, VERTICAL_FIRST=0 (the negative control: the leg must DISAPPEAR)")
    print("   argv: %s" % argv)
    if rc != 0 or "--vertical-first-m" in argv:
        bad.append("VERTICAL_FIRST=0 left the climb leg in the argv (rc=%d): %r" % (rc, argv))
    print()

    # ---------------------------------------------------------------- 2. SHAPING_M reaches the plan box
    rc, out, argv = dry_run({"SHAPING_M": "0.30"})
    print("PLANNER=rectangle with SHAPING_M=0.30")
    print("   argv: %s" % argv)
    if "--approach-shaping-m 0.30" not in argv:
        bad.append("SHAPING_M=0.30 did not reach the planner's argv: %r" % argv)
    if rc != 0:
        bad.append("SHAPING_M=0.30 makes the harness exit %d" % rc)
    print()

    # ---------------------------------------------------------------- 3. ego selects the adapter, not a copy
    rc, out, argv = dry_run({"PLANNER": "ego"})
    print("PLANNER=ego")
    print("   rc=%d   file=%s" % (rc, field(out, "file")))
    print("   argv: %s" % argv)
    if rc != 0:
        bad.append("PLANNER=ego exits %d" % rc)
    pf = field(out, "file")
    if not pf.endswith("planner/ego_source.py"):
        bad.append("PLANNER=ego does not select planner/ego_source.py (got %r)" % pf)
    if not os.path.isfile(pf):
        bad.append("PLANNER=ego names a file that does not exist: %s" % pf)
    if "--approach-shaping-m" in argv or "--vertical-first-m" in argv:
        bad.append("PLANNER=ego carries the rectangle's own flags, which it does not accept: %r" % argv)
    print()

    # ---------------------------------------------------------------- 3b. WHAT THE WIRE CARRIES (round 76)
    #     `--fields` decides which of position / velocity / acceleration the FCU receives, and the two planners
    #     need DIFFERENT answers: ego's reference has a velocity (and an acceleration the sampler cannot yet
    #     deliver smoothly -- measured, DIAG33-routeb), the rectangle's has neither (its v/a are zero by
    #     construction, so pv/pva would order the FCU to brake).  Checked in all four directions -- the frozen
    #     default, ego's new default, an override, and a value that is not a mode.
    for env, want, label in (({}, "pos", "rectangle (the frozen mission) keeps pos"),
                             ({"PLANNER": "ego"}, "posvel", "ego defaults to position + velocity"),
                             ({"PLANNER": "ego", "BRIDGE_FIELDS": "posvelacc"}, "posvelacc",
                              "and pva is one knob away"),
                             ({"TAKEOFF_COMMAND_M": "0.2", "VERTICAL_FIRST": "1"}, "pos",
                              "the promoted take-off profile does not change it")):
        rc, out, argv = dry_run(env)
        got = field(out, "wire fields").split()[0] if field(out, "wire fields") else "(absent)"
        print("wire fields, %-46s -> %s" % (label, got))
        if rc != 0:
            bad.append("%s: the harness exits %d" % (label, rc))
        if got != want:
            bad.append("wire fields: %s gave %r, expected %r" % (label, got, want))
    rc, out, _argv = dry_run({"PLANNER": "ego", "BRIDGE_FIELDS": "bogus"})
    print("wire fields, a value that is not a mode                  -> rc=%d (must be non-zero)" % rc)
    if rc == 0:
        bad.append("BRIDGE_FIELDS=bogus was accepted; a knob with three values must refuse a fourth")
    print()

    # ---------------------------------------------------------------- 3c. WHOSE SAMPLER (round 76, Route B)
    #     `EGO_CONTRACT` chooses between our adapter sampling ego's spline and ego's OWN `traj_server` doing it.
    #     The default must stay the flown path until the new one is verified on the bench, and the Route B branch
    #     must name BOTH of the processes it adds -- a missing one is a sortie that looks healthy and flies
    #     nothing.
    rc, out, argv = dry_run({"PLANNER": "ego"})
    got = re.search(r"contract (\w+)", out)
    print("ego contract, default                                   -> %s"
          % (got.group(1) if got else "(absent)"))
    if not got or got.group(1) != "adapter":
        bad.append("the default ego contract is %r, not 'adapter'" % (got.group(1) if got else None))
    rc, out, argv = dry_run({"PLANNER": "ego", "EGO_CONTRACT": "shim"})
    print("ego contract, EGO_CONTRACT=shim                         -> rc=%d file=%s"
          % (rc, os.path.basename(field(out, "file"))))
    if rc != 0:
        bad.append("EGO_CONTRACT=shim exits %d" % rc)
    if not field(out, "file").endswith("planner/ego_contract_shim.py"):
        bad.append("EGO_CONTRACT=shim does not select planner/ego_contract_shim.py (got %r)"
                   % field(out, "file"))
    for want in ("--odom-topic /ego/odom_fcu", "--trace-csv", "--arrive-m 0.30"):
        if want not in argv:
            bad.append("EGO_CONTRACT=shim's argv is missing %r: %r" % (want, argv))
    for want in ("traj_server", "fcu_pose_feed.py", "time_forward:=1.0"):
        if want not in out:
            bad.append("EGO_CONTRACT=shim does not say it would start %r" % want)
    rc, out, _argv = dry_run({"PLANNER": "ego", "EGO_CONTRACT": "bogus"})
    print("ego contract, a value that is not a mode                -> rc=%d (must be non-zero)" % rc)
    if rc == 0:
        bad.append("EGO_CONTRACT=bogus was accepted")
    print()

    # ---------------------------------------------------------------- 4. the ego stack follows the planner
    #     A planner that needs a SECOND process is the reason `EGO_STACK` exists at all, so the decision is
    #     checked in all three directions -- implied by the planner, off for the flown one, and overridable
    #     for an ego that is already running outside the harness.
    for env, want, label in (({"PLANNER": "ego"}, "1", "PLANNER=ego implies it"),
                             ({}, "0", "the flown planner does not start it"),
                             ({"PLANNER": "ego", "EGO_STACK": "0"}, "0", "and it can be overridden")):
        rc, out, argv = dry_run(env)
        got = field(out, "ego stack").split()[0] if field(out, "ego stack") else "(absent)"
        print("ego stack, %-34s -> %s" % (label, got))
        if rc != 0:
            bad.append("%s: the harness exits %d" % (label, rc))
        if got != want:
            bad.append("ego stack: %s gave %r, expected %r" % (label, got, want))
    rc, out, argv = dry_run({"PLANNER": "ego"})
    if "ego_on_lio.launch.py" not in out or "cloud_relay.py" not in out:
        bad.append("PLANNER=ego does not say it would start the ego node and the cloud relay")
    print()

    # ---------------------------------------------------------------- 5. a named file, and a refusal
    mine = os.path.join(CODE, "planner", "ego_source.py")
    rc, out, argv = dry_run({"PLANNER": mine})
    print("PLANNER=<a path>")
    print("   rc=%d   argv: %s" % (rc, argv))
    if rc != 0 or mine not in out:
        bad.append("PLANNER=<a path> did not take the path (rc=%d): %r" % (rc, out.strip()[-200:]))
    print()

    rc, out, argv = dry_run({"PLANNER": "nonsense"})
    print("PLANNER=nonsense (must be REFUSED)")
    print("   rc=%d   %s" % (rc, (out.strip().splitlines() or [""])[-1]))
    if rc == 0:
        bad.append("PLANNER=nonsense was accepted -- an unknown name must be refused, not guessed at")
    if "unknown PLANNER" not in out:
        bad.append("PLANNER=nonsense was refused without saying why")
    print()

    rc, out, argv = dry_run({"PLANNER": "/no/such/planner.py"})
    print("PLANNER=/no/such/planner.py (must be REFUSED)")
    print("   rc=%d   %s" % (rc, (out.strip().splitlines() or [""])[-1]))
    if rc == 0:
        bad.append("PLANNER=/no/such/planner.py was accepted -- a missing file must be refused")
    print()

    # ---------------------------------------------------------------- 5. no lock left behind
    if os.path.exists(LOCK):
        try:
            with open(LOCK) as fh:
                body = fh.read().strip()
        except OSError:
            body = "(unreadable)"
        if "knob-dry-run" in body:
            bad.append("the dry run left its OWN sortie lock behind (%r) -- it must exit before the lock" % body)
        else:
            print("the lock at %s belongs to another sortie (%r); the dry runs did not touch it"
                  % (LOCK, body))
    else:
        print("no sortie lock left behind")

    print()
    if bad:
        print("PLANNER-KNOB TEST FAILED (%d problem(s)):" % len(bad))
        for b in bad:
            print("   - %s" % b)
        return 1
    print("PLANNER-KNOB TEST PASSED: the default is the flown planner with the flown argv, ego selects the "
          "adapter, and every wrong name is refused before anything starts")
    return 0


if __name__ == "__main__":
    sys.exit(main())
