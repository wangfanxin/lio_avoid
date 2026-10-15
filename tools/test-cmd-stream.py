#!/usr/bin/env python3
"""The PLAN box's executor and its command stream, tested with NO flight and NO ROS graph.

WHY THIS FILE IS THE CONTRACT TEST NOW (F1, 2026-10-16).  Until F1 the stream was a shadow and this file
proved that the shadow was harmless; the thing that FLEW was the FSM's corner walk, tested by
`tools/test-corner-tour.py`.  F1 moved execution into the plan box and deleted the corner walk, so this
file inherits that test's job: it asserts the tour -- the same five waypoints, in the same order, with the
same dwells -- against the object that now flies it.

WHAT IT ASSERTS, and why each one is a flight-safety statement:

  1. every streamed position IS one of the tour's waypoints, exactly (1e-9): the stream commands
     waypoints, so "off the path by 3 mm" is no longer a possible failure -- being off a WAYPOINT is;
  2. the stream is in L: `R_WL @ p_L + t_WL` reproduces the commanded W waypoint for three calibrations;
  3. the dwell is CONTENT -- position = the corner, velocity = 0, acceleration = 0 -- and lasts
     `--hold-s`, with ONE dwell per corner of the tour and NONE on the entry leg;
  4. **THE DWELL CLOCK STARTS ON SETTLED, NOT ON THE FIRST RADIUS CROSSING.**  The driver has a "ring"
     mode that reproduces the measured behaviour (arrive at 1.7 m/s, overshoot ~0.19 m, come back
     ~3.5 s later) and the test requires the dwell to start only when the vehicle is inside `--arrive-m`
     AND slow.  This is the whole of F1 change 8: RECT11 c3 4.96 s and RECT21 c3 4.80 s both missed the
     5.0 s bar because one arrival was split in two by exactly that ring;
  5. the stream is SELF-CLOCKED at the declared rate (the S3 shadow declared 100 Hz and emitted 4);
  6. `trajectory_id` INCREMENTS PER WAYPOINT -- that field is what the consumer turns into the `leg`
     phases, and the metric window depends on there being five of them;
  7. the last message carries `COMPLETED`, and the stream does NOT go silent afterwards (the F2
     supervisor needs "finished" and "stopped talking" to be different facts);
  8. the leg clocks do not run while the vehicle is on the pad (the plan box starts ~25 s before
     take-off; a clock armed at start-up would advance the tour on the ground).

NEGATIVE CONTROLS, run by default: the dwell moved 5 mm off the corner; a stream that declares a rate it
does not emit; a stream that ends without COMPLETED; and **the settle removed** (so the dwell starts on the
raw radius crossing) -- which must fail assertion 4 and nothing else.

usage: tools/test-cmd-stream.py [--no-self-test]
"""
from __future__ import annotations

import argparse
import importlib.util
import math
import os
import sys
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from common import frames                            # noqa: E402
from planner.cmd_stream import (CmdStream, TourPacer, FRAME_L,   # noqa: E402
                                TRAJECTORY_STATUS_COMPLETED, TRAJECTORY_STATUS_READY)


def read_world(path):
    spec = importlib.util.spec_from_file_location("world_info_cmd", os.path.join(HERE, "world-info.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.read(path)


def corners_of(world):
    c = np.asarray([[q[0], q[1], q[2]] for q in world["corners"]], dtype=float)
    if len(c) > 1 and np.allclose(c[0], c[-1]):
        c = c[:-1]
    return c


def tour_targets(corners):
    """The tour exactly as `planner/rectangle_source.py::tour_targets` builds it: five waypoints."""
    corners = np.asarray(corners, dtype=float)
    return np.vstack([corners, corners[0:1]])


def drive(corners, hold_s, rate, speed, calib, arrive_m=0.06, settle_speed=0.08, settle_s=3.0,
          timeout_s=20.0, wrong=None, ring=False, on_pad_s=0.0):
    """Fly the tour on a clock this function owns, and return the samples the stream produced.

    The vehicle is a point that always moves TOWARD the waypoint the pacer is currently asking for and
    stands still while the pacer dwells, so its position is a function of the pacer's own state -- a
    simulation of the closed loop rather than of a table.

    `ring` reproduces the MEASURED arrival: the vehicle reaches the corner, overshoots ~0.25 * e^{-t/2} m
    along the leg and comes back (~0.19 m peak at 1.5 s, inside 0.06 m again at ~3.5 s).  It is what
    assertion 4 exists for.  `on_pad_s` keeps the vehicle on the ground (W altitude 0) for that long
    first, which is what the leg clocks must refuse to run through.
    """
    targets_w = tour_targets(corners)
    n_leg = len(targets_w)
    pacer = TourPacer(targets_w, hold_s, arrive_m, settle_speed, settle_s, timeout_s, 0.0,
                      seg_len_w=lambda i: float(np.linalg.norm(targets_w[i][:2] -
                                                               (targets_w[i - 1][:2] if i else
                                                                corners[0][:2]))),
                      leg_speed=speed)
    R_LW = calib.R_WL.T
    targets_l = (R_LW @ (targets_w - calib.t_WL).T).T
    stream = CmdStream(pacer, rate, targets_l, trajectory_id=1)

    if wrong == "rate":
        stream.period = 2.0 / rate
    if wrong == "settle-off":
        pacer.settle_speed_mps = 1e9          # the dwell starts on the raw radius crossing again

    k = max(1, int(math.ceil(rate / 1000.0)))         # >= 1000 pacer steps per second
    micro = (1.0 / rate) / k
    t = 0.0
    max_t = 400.0
    samples = []
    z_mission = float(corners[0][2])
    # THE PAD IS W's ORIGIN: the world file's arena is centred on it and the mission square sits at
    # x[0.5, 2.5] y[0.4, 1.6], so the entry leg is 0.64 m -- the leg the FSM's own `leg1` phase flew.
    pb = np.zeros(2)
    t_ring = None
    u_ring = np.array([1.0, 0.0])
    i_last = 0
    while t < max_t:
        # THE VEHICLE DOES NOT FLY ON THE GROUND.  The plan box starts ~25 s before take-off, and the
        # driver has to model that or the arming rule (`TourPacer._arm`) is tested against a vehicle that
        # is already taxiing.
        airborne = t >= on_pad_s
        z = z_mission if airborne else 0.0
        v_h = 0.0
        if pacer.i_leg != i_last:
            i_last = pacer.i_leg
            t_ring = None
        if airborne and not pacer.done:
            target = pacer.target_w[:2]
            dist = float(np.linalg.norm(target - pb))
            if ring and t_ring is None and dist <= arrive_m:
                t_ring = t
                u_ring = (target - pb) / max(dist, 1e-9)
            if t_ring is not None:
                # THE RING: a damped overshoot past the corner, along the direction it arrived on.  It
                # keeps running through the DWELL, because that is what the aircraft does -- the pacer's
                # opinion of where it is does not stop the motion.
                # MEASURED shape (RECT11 c3): 0.1864 m from the corner 0.86 s after the arrival, back
                # inside 0.15 m at 1.5-3.5 s -- here 0.25 * e^{-t/2} * sin(2 pi t / 4), peak 0.19 m.
                dt_ = t - t_ring
                d = 0.25 * math.exp(-dt_ / 2.0) * math.sin(2.0 * math.pi * dt_ / 4.0)
                p_new = target + u_ring * d
                v_h = float(np.linalg.norm(p_new - pb)) / micro if micro > 0 else 0.0
                pb = p_new
            elif not pacer.dwelling:
                step = speed * micro
                if dist <= step:
                    pb = target.copy()
                    v_h = 0.0
                else:
                    pb = pb + (target - pb) / dist * step
                    v_h = speed
        p_w = np.array([pb[0], pb[1], z])
        pacer.odom_n = int(t * 200) + 1
        pacer.odom_wall = t
        pacer.step(p_w, v_h, t)
        if stream.next_t is None:
            stream.next_t = t
        if stream.due(t):
            s = stream.sample()
            if wrong == "dwell-offset" and pacer.dwelling:
                s.p = np.asarray(s.p, dtype=float) + np.array([0.005, 0.0, 0.0])
            # the LAST field is the VEHICLE's horizontal distance from the waypoint the stream is
            # commanding -- what `tools/dwell-audit.py` measures from the tap on a real flight.
            samples.append((t, s, pacer.i_leg, pacer.dwelling, pacer.done,
                            float(np.linalg.norm(pb - pacer.target_w[:2]))))
            stream.advance(t)
        t += micro
        if pacer.done and t >= pacer.t_done + 0.5:
            break
    return samples, pacer, stream


def drive_vertical(corners, hold_s, rate, speed, calib, on_pad_s=0.0, vertical_m=2.5,
                   break_guard=False):
    """The SAME tour with a VERTICAL FIRST TARGET in front of it -- the setpoint-driven take-off's climb leg.

    The vehicle stands on the pad (W altitude 0) for `on_pad_s`, then moves TOWARD whatever the pacer is
    asking for in 3-D at `speed` m/s.  What this case exists for is the ARMING rule, and the flight that
    measured it is `RECT33-accept`: the climb leg's target IS the pad, so a rule phrased as "already within
    `arm_near_m` of the waypoint" arms the leg while the aircraft is parked, burns the 20 s clock through
    pre-flight, and times the leg out ~1 s after the relay opens on airborne.  `break_guard` restores that
    pre-fix behaviour (it disables exactly the guard, in the producer's own terms) so the test can show it
    catches it.
    """
    targets_w = np.vstack([np.array([[0.0, 0.0, float(vertical_m)]]), tour_targets(corners)])
    pacer = TourPacer(targets_w, hold_s, 0.06, 0.08, 3.0, 20.0, 0.0,
                      seg_len_w=lambda i: float(np.linalg.norm(targets_w[i][:2] -
                                                               (targets_w[i - 1][:2] if i else
                                                                targets_w[0][:2]))),
                      leg_speed=speed)
    if break_guard:
        # THE PRE-FIX RULE, RESTORED VERBATIM: `arm_near_m` asked of every leg, the vertical one included.
        # This is the mutation the control uses -- it is the producer's own old body, not a flag the test
        # invented, and with it `RECT33-accept`'s outcome must come back: armed at t=0 on the pad, and the
        # 20 s clock burning through pre-flight to a TIMEOUT just after the relay opens.
        def _arm_prefix(self, p_w, now):
            if p_w is None:
                return False
            if float(np.linalg.norm(p_w[:2] - self.target_w[:2])) <= self.arm_near_m:
                return True
            if float(p_w[2]) > self.arm_above_m:
                return True
            if self.p_first is not None and float(np.linalg.norm(p_w - self.p_first)) > self.arm_move_m:
                return True
            return False
        pacer._arm = types.MethodType(_arm_prefix, pacer)
    R_LW = calib.R_WL.T
    targets_l = (R_LW @ (targets_w - calib.t_WL).T).T
    stream = CmdStream(pacer, rate, targets_l, trajectory_id=1)
    k = max(1, int(math.ceil(rate / 1000.0)))
    micro = (1.0 / rate) / k
    t, max_t, samples = 0.0, 400.0, []
    logs = []
    pacer.log = logs.append
    pb = np.zeros(3)
    i_last = 0
    armings = []                      # (t, leg index) every time a leg's flight clock started
    was_armed = False
    while t < max_t:
        if pacer.i_leg != i_last:
            i_last = pacer.i_leg
        if t >= on_pad_s and not pacer.done and not pacer.dwelling:
            tgt = np.asarray(pacer.target_w, dtype=float)
            dv = tgt - pb
            dist = float(np.linalg.norm(dv))
            step = speed * micro
            pv = 0.0
            if dist <= step:
                pb = tgt.copy()
            else:
                pb = pb + dv / dist * step
                pv = speed
            pacer.step(pb, pv, t)
        else:
            pacer.step(pb.copy(), 0.0, t)
        pacer.odom_n = int(t * 200) + 1
        pacer.odom_wall = t
        if pacer.armed and not was_armed:
            armings.append((t, pacer.i_leg))
        was_armed = pacer.armed
        if stream.next_t is None:
            stream.next_t = t
        if stream.due(t):
            s = stream.sample()
            samples.append((t, s, pacer.i_leg, pacer.dwelling, pacer.done))
            stream.advance(t)
        t += micro
        if pacer.done and t >= pacer.t_done + 0.5:
            break
    return samples, pacer, stream, logs, targets_w, armings


def group_samples(samples):
    """The samples grouped into maximal runs of one (waypoint, dwelling, done) state."""
    groups = []
    for row in samples:
        key = (row[2], row[3], row[4])
        if groups and groups[-1][0] == key:
            groups[-1][1].append(row)
        else:
            groups.append((key, [row]))
    return groups


def run_checks(samples, corners, hold_s, rate, calib, quiet=False, ring=False):
    problems = []
    targets_w = tour_targets(corners)

    def ok(cond, msg):
        if not cond:
            problems.append(msg)
            if not quiet:
                print("   FAIL %s" % msg)
        return cond

    def to_w(p_l):
        return calib.R_WL @ np.asarray(p_l, dtype=float) + calib.t_WL

    # ---- 1. every streamed position IS a waypoint of the tour, exactly -----------------------------
    worst = 0.0
    for t, s, i_leg, dwell, done, d_self in samples:
        p_w = to_w(s.p)
        worst = max(worst, float(np.linalg.norm(p_w - targets_w[min(i_leg, len(targets_w) - 1)])))
    ok(worst <= 1e-9, "a streamed position is not the waypoint it claims: worst %.3e m" % worst)

    # ---- 2. the stream is in L ----------------------------------------------------------------
    ok(all(s[1].frame_id == FRAME_L for s in samples),
       "a sample's frame_id is not %r" % FRAME_L)

    # ---- 3. the dwell is content, and there is one per corner of the tour, none on the entry ------
    groups = group_samples(samples)
    dwells = [(k, g) for k, g in groups if k[1] and not k[2]]
    ok(len(dwells) == len(targets_w) - 1,
       "expected %d corner dwells (the tour's corners; the entry waypoint has none), got %d"
       % (len(targets_w) - 1, len(dwells)))
    for k, g in dwells:
        i_leg = k[0]
        corner = targets_w[i_leg]
        span = g[-1][0] - g[0][0]
        off = max(float(np.linalg.norm(to_w(r[1].p) - corner)) for r in g)
        vmax = max(float(np.linalg.norm(r[1].v)) for r in g)
        amax = max(float(np.linalg.norm(r[1].a)) for r in g)
        ok(span >= hold_s - 1.0 / rate - 1e-9,
           "dwell at waypoint %d lasted %.4f s < hold_s %.4f s (one sample period allowed)"
           % (i_leg + 1, span, hold_s))
        ok(off <= 1e-9, "dwell at waypoint %d is %.3e m off the corner" % (i_leg + 1, off))
        ok(vmax == 0.0, "dwell at waypoint %d carries velocity %.6g, not 0" % (i_leg + 1, vmax))
        ok(amax == 0.0, "dwell at waypoint %d carries acceleration %.6g, not 0" % (i_leg + 1, amax))

    # ---- 4. THE DWELL CLOCK STARTS ON SETTLED (F1 change 8) ----------------------------------------
    #
    # The statement that matters is not about the COMMAND (which always says "the corner") but about the
    # VEHICLE: `tools/dwell-audit.py` measures, from the truth tap, the longest stretch during which the
    # aircraft is inside a 0.15 m radius of the corner, and requires >= 5.0 s.  The same measurement is
    # made here on the driver's vehicle, so the RING control -- the measured RECT11 c3 behaviour -- fails
    # when the dwell clock starts on the raw radius crossing instead of on the settled arrival.
    RADIUS = 0.15
    for k, g in dwells:
        d0 = g[0][5]
        ok(d0 <= 0.06 + 1e-9,
           "the dwell at waypoint %d STARTED with the vehicle %.4f m from the corner (> --arrive-m)"
           % (k[0] + 1, d0))
        # the longest CONTIGUOUS stretch with the vehicle inside the judging radius, in samples
        best = run = 0
        for row in g:
            run = run + 1 if row[5] <= RADIUS else 0
            best = max(best, run)
        flown = best / rate
        ok(flown >= hold_s - 1.0 / rate - 1e-9,
           "the FLOWN dwell at waypoint %d is %.3f s inside the %.2f m radius, shorter than the %.1f s "
           "commanded: the dwell clock is running from the first radius crossing, so the arrival ring "
           "spends part of it outside the radius" % (k[0] + 1, flown, RADIUS, hold_s))
    if ring:
        starts = [g[0][0] for _, g in dwells]
        ok(min(starts) >= 2.0, "with a ring, the first dwell started at %.3f s -- the vehicle cannot have "
                               "settled by then" % min(starts))

    # ---- 5. self-clocked --------------------------------------------------------------------------
    dev, n_pairs = 0.0, 0
    for k, g in groups:
        for a, b in zip(g, g[1:]):
            dev = max(dev, abs((b[0] - a[0]) - 1.0 / rate))
            n_pairs += 1
    ok(n_pairs > 0 and dev <= 1e-6,
       "the stream is not self-clocked at %g Hz: worst spacing error %.3e s over %d pairs"
       % (rate, dev, n_pairs))
    total = samples[-1][0] - samples[0][0]
    measured = (len(samples) - 1) / total if total > 0 else float("inf")
    ok(abs(measured - rate) <= 0.02 * rate,
       "the stream's own rate is %.3f Hz, declared %g Hz (2%% band)" % (measured, rate))

    # ---- 6. trajectory_id increments once per waypoint, and never 0 -------------------------------
    ids = [s[1].trajectory_id for s in samples]
    ok(all(i != 0 for i in ids), "a sample carries trajectory_id 0, which the message documents as "
                                 "'no trajectory yet'")
    ok(ids == sorted(ids), "trajectory_id goes backwards: %s" % ids[:8])
    ok(len(set(ids)) == len(targets_w),
       "expected %d distinct trajectory_ids (one per waypoint -- they are what the consumer turns into "
       "the leg phases), got %d" % (len(targets_w), len(set(ids))))

    # ---- 7. COMPLETED last, and the stream does not go silent -------------------------------------
    ok(samples[-1][1].completed, "the last sample's trajectory_flag is %d, not COMPLETED(%d)"
       % (samples[-1][1].trajectory_flag, TRAJECTORY_STATUS_COMPLETED))
    done_groups = [(k, g) for k, g in groups if k[2]]
    ok(len(done_groups) == 1 and len(done_groups[0][1]) >= 10,
       "the COMPLETED statement is not a sustained stream (%d group(s)): the F2 supervisor must be able "
       "to tell 'finished' from 'stopped talking'" % len(done_groups))
    ok(all(s[1].trajectory_flag == TRAJECTORY_STATUS_READY
           for s in samples if not s[4]), "a non-final sample does not carry READY")
    ok(all(list(s[1].kx) == [0.0] * 3 and list(s[1].kv) == [0.0] * 3 for s in samples),
       "kx/kv are not zero: the message has no gain fields on the wire (msg 84) and ego also sends zeros")
    return problems


def self_test(corners, args, calib):
    """The negative controls: mutate the PRODUCER (never the test) and require a failure."""
    ok = True
    controls = [
        ("dwell position moved 5 mm off the corner", dict(wrong="dwell-offset"), "off the corner"),
        ("a rate declared at 100 Hz but emitted at 50 Hz", dict(wrong="rate"), "self-clocked"),
        ("a stream that never says COMPLETED", dict(wrong="nocomplete"), "COMPLETED"),
        ("the SETTLE removed: the dwell starts on the raw radius crossing",
         dict(wrong="settle-off", ring=True), "FLOWN dwell"),
    ]
    for label, kw, needle in controls:
        print()
        print("NEGATIVE CONTROL: %s" % label)
        samples, pacer, stream = drive(corners, args.hold_s, args.rate, args.speed, calib,
                                      ring=kw.pop("ring", False), **kw)
        if kw.get("wrong") == "nocomplete":
            # keep only what the stream said BEFORE the tour was declared done: it then ends READY, which
            # is exactly "the producer stops talking without saying it finished"
            samples = [s for s in samples if not s[4]]
        if not samples:
            raise SystemExit("negative control %r produced no samples -- control is stale" % label)
        problems = run_checks(samples, corners, args.hold_s, args.rate, calib, quiet=True,
                              ring=False)
        hit = [p for p in problems if needle in p]
        print("   the mutated producer produced %d sample(s) and the contract test reported %d "
              "problem(s)" % (len(samples), len(problems)))
        print("   verdict: %s" % ("FAILS as required" if hit else "DID NOT FAIL -- the control is dead"))
        for p in (hit or problems)[:2]:
            print("      %s" % p)
        if not hit:
            ok = False
    # the vertical-leg guard, mutated back to the pre-fix rule: the climb leg's clock must then arm ON THE
    # PAD and time the leg out, which is what `RECT33-accept` measured
    print()
    print("NEGATIVE CONTROL: the vertical leg allowed to arm on `arm_near_m` (the pre-fix rule)")
    _s, pacer_b, _st, logs_b, _vt, arm_b = drive_vertical(corners, args.hold_s, args.rate, args.speed,
                                                          calib, on_pad_s=25.0, break_guard=True)
    timed_out = [s for s in logs_b if "TIMEOUT" in s]
    armed_on_pad = bool(arm_b) and arm_b[0][0] < 25.0
    print("   the mutated producer armed leg %d at t=%.3f s and produced %d timeout line(s)"
          % ((arm_b[0][1] + 1) if arm_b else 0, arm_b[0][0] if arm_b else float("nan"), len(timed_out)))
    print("   verdict: %s" % ("FAILS as required" if (timed_out and armed_on_pad)
                              else "DID NOT FAIL -- the control is dead"))
    if timed_out:
        print("      %s" % timed_out[0][:160])
    if not (timed_out and armed_on_pad):
        ok = False
    return ok


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default=os.path.join(ROOT, "worlds", "pillars.yaml"))
    ap.add_argument("--rate", type=float, default=100.0, help="the stream's declared rate, Hz")
    ap.add_argument("--hold-s", type=float, default=5.0)
    ap.add_argument("--speed", type=float, default=0.50)
    ap.add_argument("--self-test", dest="self_test", action="store_true", default=True,
                    help="run the negative controls (default: on -- a control that is not run by default "
                         "is a control that stops being run)")
    ap.add_argument("--no-self-test", dest="self_test", action="store_false")
    args = ap.parse_args()

    world = read_world(args.world)
    corners = corners_of(world)
    targets = tour_targets(corners)
    print("world %s: %d corners -> a tour of %d waypoints (entry = corner 1)" %
          (os.path.basename(args.world), len(corners), len(targets)))

    # THREE calibrations, because "the stream is in L" must not be a claim about one file: the measured
    # one the flight used, the prediction the producer falls back to, and a deliberately rotated one.
    cals = [frames.Calib.predicted(spawn_heading_deg=0.0, world_axes="body"),
            frames.Calib.predicted(spawn_heading_deg=37.0, world_axes="body"),
            frames.Calib(R_WL=frames.R_z(math.radians(123.0)), t_WL=np.array([0.31, -0.22, 0.07]))]
    measured = "/mnt/d/distill_self/_stage/calib.json"
    if os.path.exists(measured):
        import json
        d = json.load(open(measured))
        cals.insert(0, frames.Calib(R_WL=np.array(d["R_WL"]), t_WL=np.array(d["t_WL"])))
        print("using the MEASURED calibration from %s (yaw %.4f deg) as case 1"
              % (measured, d.get("yaw_deg", float("nan"))))

    problems = []
    for i, calib in enumerate(cals, start=1):
        print()
        print("=== calibration %d/%d: yaw %+.4f deg, t = [%+.4f %+.4f %+.4f] m"
              % (i, len(cals), math.degrees(math.atan2(calib.R_WL[1, 0], calib.R_WL[0, 0])),
                 *calib.t_WL))
        samples, pacer, stream = drive(corners, args.hold_s, args.rate, args.speed, calib)
        for key, g in group_samples(samples):
            kind = "COMPLETED" if key[2] else ("dwell  " if key[1] else "fly    ")
            print("   waypoint %d %s %6d samples  t=%8.3f..%8.3f  span %6.3f s"
                  % (key[0] + 1, kind, len(g), g[0][0], g[-1][0], g[-1][0] - g[0][0]))
        print("   %s" % stream.summary())
        print("   pacer: done=%s, waypoints commanded=%d of %d" % (pacer.done, pacer.i_leg + 1,
                                                                   len(targets)))
        if not pacer.done:
            problems.append("calibration %d: the tour never completed (pacer.i_leg=%d)"
                            % (i, pacer.i_leg))
        problems += ["calib %d: %s" % (i, p) for p in
                     run_checks(samples, corners, args.hold_s, args.rate, calib)]

    # ---- 8. the ring: the dwell clock must wait for the vehicle to be SLOW, not merely inside ------
    print()
    print("=== the MEASURED arrival ring (RECT11 c3 / RECT21 c3): 0.25 m overshoot, back inside "
          "0.06 m at ~3.5 s")
    samples, pacer, stream = drive(corners, args.hold_s, args.rate, args.speed, cals[0], ring=True)
    for key, g in group_samples(samples):
        if key[1] and not key[2]:
            print("   dwell at waypoint %d starts at t=%.3f s, %.4f m from the corner"
                  % (key[0] + 1, g[0][0], g[0][5]))
    problems += ["ring: %s" % p for p in
                 run_checks(samples, corners, args.hold_s, args.rate, cals[0], ring=True)]

    # ---- 9. the clocks do not run while the vehicle is on the pad ---------------------------------
    print()
    print("=== on the pad for 30 s before take-off (the plan box starts ~25 s before the aircraft moves)")
    samples, pacer, stream = drive(corners, args.hold_s, args.rate, args.speed, cals[0], on_pad_s=30.0)
    early = [s for s in samples if s[0] < 30.0]
    moved = sorted({s[2] for s in early})
    if moved != [0]:
        problems.append("the tour ADVANCED while the vehicle was on the pad: waypoints %s commanded "
                        "before take-off" % moved)
    else:
        print("   ok: %d samples in the first 30 s, all commanding waypoint 1 (no leg clock ran, and no "
              "leg timeout fired, while the aircraft sat on the pad)" % len(early))

    # ---- 10. a VERTICAL first target must not arm on the pad either --------------------------------
    print()
    print("=== a VERTICAL first target (the setpoint-driven take-off's climb leg), 25 s on the pad first")
    _s, pacer_v, _st, logs_v, vt, arm_v = drive_vertical(corners, args.hold_s, args.rate, args.speed,
                                                         cals[0], on_pad_s=25.0)
    on_pad = [r for r in _s if r[0] < 25.0]
    moved_v = sorted({r[2] for r in on_pad})
    print("   %d waypoints commanded: %s (first is U +%.2f m straight up, then the %d corners)"
          % (len(vt), ", ".join("(%.2f, %.2f, %.2f)" % tuple(q) for q in vt[:3]) + ", ...",
             vt[0][2], len(vt) - 1))
    print("   arming: %s; %d samples before take-off"
          % ("; ".join("leg %d at t=%.3f s" % (i + 1, tt) for tt, i in arm_v) or "never",
             len(on_pad)))
    if moved_v != [0]:
        problems.append("the tour ADVANCED past the VERTICAL leg while the vehicle was on the pad: "
                        "waypoints %s commanded before take-off" % moved_v)
    early = [(tt, i) for tt, i in arm_v if tt < 25.0]
    if early:
        problems.append("a VERTICAL leg's flight clock ARMED ON THE PAD at t=%.3f s (leg %d): its target IS "
                        "the pad, so `arm_near_m` cannot be asked of it (RECT33-accept timed the leg out "
                        "0.9 s after the relay opened, with the aircraft at 0.68 m of the commanded 2.50 m)"
                        % (early[0][0], early[0][1] + 1))
    if any("TIMEOUT" in s for s in logs_v):
        problems.append("the VERTICAL leg timed out: %s"
                        % [s for s in logs_v if "TIMEOUT" in s][0][:120])
    if not any("ARRIVED at waypoint 1" in s for s in logs_v):
        problems.append("the VERTICAL leg never arrived (the climb leg must arrive at the mission altitude)")
    if not pacer_v.done:
        problems.append("the tour did not complete behind the vertical leg (i_leg=%d of %d)"
                        % (pacer_v.i_leg, len(vt)))
    if not problems:
        print("   ok: no leg armed before take-off (first at t=%.3f s), the climb ARRIVED, the tour ran"
              % (arm_v[0][0] if arm_v else float("nan")))

    ok = not problems
    if args.self_test:
        ok = self_test(corners, args, cals[0]) and ok

    print()
    if problems:
        print("CMD-STREAM TEST FAILED (%d problem(s)):" % len(problems))
        for p in problems[:12]:
            print("   - %s" % p)
    print("CMD-STREAM TEST %s" % ("PASSED" if ok else "FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
