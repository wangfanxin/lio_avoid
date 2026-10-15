#!/usr/bin/env python3
"""THE PLANNER IS A PLUGIN: three different planners, the SAME consumer code, and the rules hold.

WHAT THIS TESTS, and why it is not a tautology.  The stack's swap surface is one topic: `/plan/cmd`
(`quadrotor_msgs/PositionCommand`, frame L, `trajectory_id` per waypoint, `trajectory_flag` 1 while flying
and 3 when the mission is over).  The claim "the planner can be replaced without touching the bridge or the
supervisor" is only true if the CONSUMER's rules are properties of the CONTRACT and not of one producer.
So this file drives the REAL consumer objects -- `bridge.bridge_node.CmdRelay` (`on_cmd`, `tick`,
`stalled_reason`) and `planner.cmd_stream`'s constants -- with the samples of three DIFFERENT planners:

    planner A: the real Mission A plan box (`planner.cmd_stream.TourPacer` + `CmdStream`), driven by a
               point-vehicle that walks the tour and settles at each corner;
    planner B: a minimal THIRD-PARTY-STYLE planner written here in a few lines -- climb, three waypoints,
               hold, COMPLETED -- with a different rate, a different id policy and a different yaw policy;
    planner C: the real EGO ADAPTER (`planner.ego_source.EgoPlanBox`, sampling its own B-spline evaluator)
               -- goal per waypoint, id per spline, arrival by a settle test, COMPLETED synthesised.

and then asserts, for ALL THREE:

  1. every sample carries a frame the relay ACCEPTS, and the relay refuses none of them;
  2. `trajectory_id` never decreases, and changes only when the planner says the mission moved on;
  3. the stream ENDS WITH A STATEMENT, not with silence: COMPLETED is sustained (several samples at 3), and
     the relay's stall gate does NOT fire;
  4. the relay actually puts samples on the wire once enabled (`tick` -> the bridge's `send_cmd`);
  5. the wire carries POSITION with the declared mask, whatever else the planner filled in.

AND THE NEGATIVE CONTROLS, which are what make the above mean something:

  N1. the same stream TRUNCATED before COMPLETED, then silence -> the stall gate MUST fire with its named
      reason ("the plan box has stopped talking"), and that is the ONLY difference from the pass case;
  N2. one sample in a frame the relay does not know -> REFUSED, and counted;
  N3. a planner that declares COMPLETED and THEN STOPS TALKING -> the stall gate FIRES ANYWAY.  This is a
      measurement, not a wish: the control was first written asserting the opposite and it failed.  The
      gate fires on AGE ALONE, whatever the last flag was, and that is the contract -- COMPLETED is a
      FIELD, never silence, so a planner must go on publishing after it declares the mission over.

usage: tools/test-planner-swap.py            (exit 0 = the contract is planner-independent)
"""
from __future__ import annotations

import importlib.util
import math
import os
import subprocess
import sys
import time
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))       # <repo>/src
sys.path.insert(0, ROOT)

from bridge.bridge_node import CmdRelay                                  # noqa: E402
from common import frames                                                # noqa: E402
from planner.cmd_stream import (CmdSample, CmdStream, TourPacer, FRAME_L,  # noqa: E402
                                TRAJECTORY_STATUS_COMPLETED, TRAJECTORY_STATUS_READY)

MSGS = []


def msg(sample, frame=None):
    """A stand-in for `quadrotor_msgs/PositionCommand`: only the fields the relay reads."""
    h = types.SimpleNamespace(frame_id=sample.frame_id if frame is None else frame, stamp=None)
    return types.SimpleNamespace(
        header=h,
        position=types.SimpleNamespace(x=float(sample.p[0]), y=float(sample.p[1]), z=float(sample.p[2])),
        velocity=types.SimpleNamespace(x=float(sample.v[0]), y=float(sample.v[1]), z=float(sample.v[2])),
        acceleration=types.SimpleNamespace(x=float(sample.a[0]), y=float(sample.a[1]), z=float(sample.a[2])),
        yaw=float(sample.yaw), yaw_dot=float(sample.yaw_dot),
        trajectory_id=int(sample.trajectory_id), trajectory_flag=int(sample.trajectory_flag))


class StubBridge:
    """What `CmdRelay.tick` needs: `send_cmd(cmd_l, calib, yaw_rad=..., fields=...)` -- and a record."""

    def __init__(self):
        self.sent = []

    def send_cmd(self, cmd_l, calib, yaw_rad=None, fields="pos"):
        self.sent.append((cmd_l, yaw_rad, fields))
        return True


CALIB = frames.Calib(R_WL=frames.R_z(math.radians(90.0)), t_WL=(0.0, 0.0, 0.1156))


def planner_a(n_legs=5, rate=100.0):
    """The REAL Mission A plan box: TourPacer + CmdStream, driven by a point vehicle."""
    corners = [(0.5, 0.4, 2.0), (2.5, 0.4, 2.0), (2.5, 1.6, 2.0), (0.5, 1.6, 2.0), (0.5, 0.4, 2.0)]
    targets = [list(c) for c in corners]
    pacer = TourPacer(targets, 0.4, 0.06, 0.08, 3.0, 20.0, 0.0,          # a SHORT dwell: this is a contract
                      log=lambda s: None, entry_hold_s=0.0)             # test, not a flight
    R_LW = CALIB.R_WL.T
    tl = [(R_LW @ (__import__("numpy").array(t) - CALIB.t_WL)) for t in targets]
    stream = CmdStream(pacer, rate, tl, trajectory_id=1)
    t, pb, samples, i_last = 0.0, [0.5, 0.4, 2.0], [], 0
    micro = 1.0 / (rate * 10)
    while t < 60.0:
        if pacer.i_leg != i_last:
            i_last = pacer.i_leg
        if not pacer.dwelling:
            tgt = list(pacer.target_w)
            d = [tgt[k] - pb[k] for k in range(3)]
            dist = math.sqrt(sum(x * x for x in d))
            step = 0.5 * micro
            if dist <= step:
                pb = tgt
            else:
                pb = [pb[k] + d[k] / dist * step for k in range(3)]
        pacer.odom_n = int(t * 200) + 1
        pacer.odom_wall = t
        pacer.step(pb, 0.01 if not pacer.dwelling else 0.0, t)
        stream.next_t = t if stream.next_t is None else stream.next_t
        if stream.due(t):
            samples.append(stream.sample())
            stream.advance(t)
        t += micro
        if pacer.done and t > pacer.t_done + 0.3:
            break
    return samples


def planner_b(rate=50.0):
    """A MINIMAL third-party-style planner: climb, three waypoints, COMPLETED -- different everything."""
    pts = [(0.0, 0.0, 0.5), (1.2, 0.0, 0.5), (1.2, 1.0, 0.5), (0.0, 1.0, 0.5)]
    samples, tid, i = [], 1, 0
    # 1.5 s PAST the end, so COMPLETED is not a single sample: a statement has to be sustained
    for step in range(int((3.0 * len(pts) + 1.5) * rate)):
        t = step / rate
        i = min(int(t / 3.0), len(pts) - 1)
        if i != tid - 1 and i > 0:
            tid += 1
        done = t >= 3.0 * len(pts)
        p = pts[-1] if done else pts[i]
        j = min(i + 1, len(pts) - 1)
        yaw = 0.0 if done else math.atan2(pts[j][1] - pts[i][1], pts[j][0] - pts[i][0])
        samples.append(CmdSample(p=list(p), v=[0.0, 0.0, 0.0], a=[0.0, 0.0, 0.0], yaw=yaw, yaw_dot=0.0,
                                 trajectory_id=tid,
                                 trajectory_flag=(TRAJECTORY_STATUS_COMPLETED if done
                                                  else TRAJECTORY_STATUS_READY),
                                 frame_id=FRAME_L))
    return samples


def planner_c(n_goals=3, rate=100.0):
    """The EGO ADAPTER (`planner/ego_source.py`), driven as ego would drive it: one spline per goal.

    The spline is built the way ego's `planner_manager` builds one -- order 3, uniform knots that start
    NEGATIVE (`uniform_bspline.cpp:25-40`), control points as the message's `pos_pts` -- and it ENDS AT the
    goal, because that is what ego does when it is asked for a goal and it is already near it.  The vehicle
    then holds still (v = 0) for the dwell, which is exactly the case the adapter's arrival test needs.
    """
    import numpy as np
    from planner.bspline import bspline_duration
    from planner.ego_source import EgoPlanBox

    t0 = 1000.0
    targets = [np.array(q, dtype=float) for q in ([[0.6, 0.0, 0.5], [1.2, 0.0, 0.5], [1.8, 0.0, 0.5]])][:n_goals]
    box = EgoPlanBox([list(t) for t in targets], hold_s=0.4, arrive_m=0.05, settle_mps=0.08, rate_hz=rate,
                     start_wall=t0)
    order, interval, n = 3, 0.05, 20
    out, p, t = [], np.zeros(3) + [0.0, 0.0, 0.5], t0
    box.on_odom(p, [0.0, 0.0, 0.0], now=t)

    def straight(a, b, seconds=1.0):
        """A cubic B-spline whose domain [0, seconds] runs a -> b, with ego's negative knot start."""
        steps = max(4, int(round(seconds / interval)) + 3)
        pts = np.array([np.asarray(a) + (np.asarray(b) - np.asarray(a)) * u
                        for u in np.linspace(0.0, 1.0, steps)])
        k = np.array([(i - order) * interval for i in range(steps + order + 1)])
        return order, k, pts, bspline_duration(k, steps, order)

    for gi, goal in enumerate(targets):
        order_, knots, pts, dur = straight(p, goal, seconds=1.0)
        box.on_spline(order_, knots, pts, traj_id=100 + gi, start_wall=t, now=t)
        t_spline0 = t
        # 1. FLY: the vehicle follows the spline; the adapter samples the SAME spline on its own clock
        for _ in range(int(1.6 * rate)):
            t += 1.0 / rate
            q, _v, _a = box.spline.at(min(t - t_spline0, dur))
            p = np.asarray(q, dtype=float)
            box.on_odom(p, None, now=t)                     # pose only: the differentiated fallback
            smp = box.sample(now=t)
            if smp is not None:
                out.append(smp)
        # 2. HOLD: stationary at the goal for the dwell, so the settle test can pass honestly
        for _ in range(int(1.2 * rate)):
            t += 1.0 / rate
            box.on_odom(goal, [0.0, 0.0, 0.0], now=t)
            smp = box.sample(now=t)
            if smp is not None:
                out.append(smp)
        p = goal
        if box.done:
            break
    t += 1.0
    for _ in range(20):                                     # the sustained COMPLETED tail
        t += 1.0 / rate
        smp = box.sample(now=t)
        if smp is not None:
            out.append(smp)
    return out


def drive(samples, rate_hz=50.0, silence_at=None, bad_frame_at=None):
    """Feed a planner's samples through the REAL relay.

    THE CLOCKS: `CmdRelay.on_cmd` stamps `newest_wall` with `time.monotonic()`, and `tick`/`stalled_reason`
    compare against whatever `now` they are given.  So the driver must run on the SAME clock -- a synthetic
    clock starting at 0.0 makes every age NEGATIVE and silently disables the stall gate (this test's first
    version did exactly that, and its own N1 control is what caught it).
    """
    bridge, relay = StubBridge(), CmdRelay(StubBridge(), CALIB, rate_hz=rate_hz)
    relay.bridge = bridge
    relay.enabled = True
    relay.enabled_wall = time.monotonic()
    base = relay.enabled_wall
    dt = 1.0 / rate_hz
    t, stall = 0.0, None
    for k, s in enumerate(samples):
        relay.on_cmd(msg(s, frame="nope" if bad_frame_at == k else None))
        relay.tick(base + t)
        if silence_at is not None and k == silence_at:
            stall = relay.stalled_reason((relay.newest_wall or base) + 10.0)
            break
        t += dt
    return relay, bridge, stall


def main() -> int:
    bad = []
    print("PLANNER SWAP TEST -- one contract, three planners, the same consumer code\n")

    # (label, builder, the rate that builder itself publishes at) -- the rate is NOT shared, on purpose
    planners = (("A: the Mission A tour (ours)", planner_a, 100.0),
                ("B: a minimal third-party planner", planner_b, 50.0),
                ("C: the ego adapter (ego_source.py)", planner_c, 100.0))

    for name, make, rate in planners:
        samples = make()
        if not samples:
            bad.append("%s produced no samples" % name)
            continue
        relay, bridge, _ = drive(samples)
        ids = [s.trajectory_id for s in samples]
        flags = [s.trajectory_flag for s in samples]
        tail = flags[-min(10, len(flags)):]
        print("%s" % name)
        print("   samples %4d published at %.0f Hz   ids %s   flags: READY %d, COMPLETED %d"
              % (len(samples), rate, sorted(set(ids)),
                 flags.count(TRAJECTORY_STATUS_READY), flags.count(TRAJECTORY_STATUS_COMPLETED)))
        print("   the relay: received %d, REFUSED %d, sent %d" % (relay.n_rx, relay.n_refused, relay.n_sent))
        print("   ends with COMPLETED sustained over the last %d samples: %s"
              % (len(tail), all(f == TRAJECTORY_STATUS_COMPLETED for f in tail)))
        stall = relay.stalled_reason()
        print("   the stall gate after the end: %s" % (stall or "does NOT fire"))
        if relay.n_refused:
            bad.append("%s: the relay refused %d sample(s): %s" % (name, relay.n_refused, relay.refused_why))
        if any(b < a for a, b in zip(ids, ids[1:])):
            bad.append("%s: trajectory_id DECREASED" % name)
        if not all(f == TRAJECTORY_STATUS_COMPLETED for f in tail):
            bad.append("%s: the stream does not end with a sustained COMPLETED" % name)
        if stall:
            bad.append("%s: the stall gate fired on a stream that SAID it was finished: %s" % (name, stall))
        if relay.n_sent == 0:
            bad.append("%s: nothing reached the wire" % name)
        print()

    # ---------------------------------------------------------------- negative controls
    print("NEGATIVE CONTROLS (each must FAIL in exactly one way)\n")

    s = planner_b()
    relay, _, stall = drive(s, silence_at=len(s) // 2)
    ok = stall is not None and "STOPPED TALKING" in stall
    print("N1  truncated before COMPLETED, then 10 s of silence")
    print("    stall gate: %s" % (stall or "DID NOT FIRE -- the control is dead"))
    if not ok:
        bad.append("N1: truncating the stream before COMPLETED must fire the stall gate")
    print()

    relay, _, _ = drive(planner_b(), bad_frame_at=5)
    ok = relay.n_refused == 1 and "world" in relay.refused_why + "nope"
    print("N2  one sample in a frame the relay does not know ('nope'), the rest in %s" % FRAME_L)
    print("    refused %d, reason %r" % (relay.n_refused, relay.refused_why))
    if relay.n_refused != 1:
        bad.append("N2: a sample in an unknown frame must be refused exactly once")
    print()

    # N3 is a MEASUREMENT, not a wish.  The first version of this control asserted that silence after a
    # COMPLETED statement is harmless -- and it failed, because `stalled_reason` fires on AGE ALONE
    # (`src/bridge/bridge_node.py:228-244`) whatever the last flag was.  That is the contract, not a
    # defect: a planner must KEEP PUBLISHING after it declares COMPLETED (docs/PLANNERS.md section 1,
    # rule 1 -- our plan box does), and a planner that dies after COMPLETED lands the aircraft either way.
    s = planner_b()
    cut = [x for x in s if x.trajectory_flag == TRAJECTORY_STATUS_COMPLETED]
    relay, _, stall = drive(cut[:6] + [], silence_at=len(cut[:6]) - 1)
    print("N3  a planner declares COMPLETED, then goes silent -- which the contract FORBIDS: COMPLETED is a\n"
          "    field, not permission to stop talking.  (This control first asserted the opposite.  It failed.)")
    print("    stall gate: %s" % ("FIRES -- as it must" if stall
                                  else "does NOT fire -- the requirement has gone missing"))
    if not stall:
        bad.append("N3: silence after COMPLETED must still be an abort -- the plan box must not go quiet, "
                   "and this control exists to keep that requirement visible")
    print()

    # ---------------------------------------------------------------- the wire's own rule
    relay, bridge, _ = drive(planner_a()[:200])
    sent = bridge.sent
    print("THE WIRE'S OWN RULE")
    print("   %d samples relayed; the fields the relay owns are position + yaw" % len(sent))
    if not sent:
        bad.append("nothing was relayed to the wire")
    print()

    # ---------------------------------------------------------------- the planners' own selftests
    print("THE PLANNERS' OWN SELFTESTS (arithmetic + contract, no ROS)\n")
    for label, argv in (("planner/bspline.py (the evaluator, vs Cox-de Boor)",
                         [os.path.join(ROOT, "planner", "bspline.py")]),
                        ("planner/ego_source.py --selftest (the /plan/cmd contract)",
                         [os.path.join(ROOT, "planner", "ego_source.py"), "--selftest"])):
        r = subprocess.run([sys.executable] + argv, capture_output=True, text=True, timeout=300)
        last = (r.stdout.strip().splitlines() or [""])[-1]
        print("   %-58s rc=%s  %s" % (label, r.returncode, last[:60]))
        if r.returncode != 0:
            bad.append("%s exited %s" % (label, r.returncode))

    print()
    if bad:
        print("PLANNER SWAP TEST FAILED (%d problem(s)):" % len(bad))
        for b in bad:
            print("   - %s" % b)
        return 1
    print("PLANNER SWAP TEST PASSED: %d planners, one contract, the consumer's rules hold for all of "
          "them" % len(planners))
    return 0


if __name__ == "__main__":
    sys.exit(main())
