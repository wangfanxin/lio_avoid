#!/usr/bin/env python3
"""The command stream's own contract, tested offline -- no ROS, no graph, no airframe, no flight.

WHAT THIS PROVES, and why it is a test rather than a demonstration.  `planner/cmd_stream.py` is the PLAN
box's output (`quadrotor_msgs/msg/PositionCommand` on `/plan/cmd`, refactor-bridge-plan.md section 3.3).
The claim the interface makes is not "a topic exists": it is that the stream says WHERE the plan wants the
vehicle, that the corner dwell is CONTENT (`position = corner, velocity = 0`) rather than a gap, that the
stream keeps its own declared rate, and that the end of the tour is a FIELD
(`trajectory_flag = TRAJECTORY_STATUS_COMPLETED`) rather than the empty-Path convention.  Every one of
those is a statement about numbers, so every one of them is checkable here.

  1. ON THE COMMANDED PATH, within 1 mm.  Every flown-phase sample lies on the leg it belongs to (the
     densified leg geometry, the same `legs_w` the Path is built from);
  2. IN L, AND CONSISTENT WITH WHAT IS FLOWN.  `position` is in L, and `R_WL @ p_L + t_WL` equals the W
     corner the flight actually commands -- with the MEASURED calibration from `_stage/calib.json` and
     with two different ones, so the frame claim is not a claim about one file;
  3. THE DWELL IS CONTENT.  At every corner there is a segment of >= `hold_s` whose `position` is the
     corner (<= 1 mm) and whose `velocity` and `acceleration` are ZERO;
  4. THE STREAM IS SELF-CLOCKED.  All the samples of a segment are spaced at the declared rate's period
     (a fake clock is driven at exactly that period, so the count per simulated second is a number this
     test can read off);
  5. THE LAST MESSAGE CARRIES COMPLETED, and `EMPTY` is what precedes the first leg;
  6. NEGATIVE CONTROLS: the same checks, run against deliberately wrong variants of the producer, must
     FAIL.  A dwell moved by 5 mm, a rate declared but not kept, a stream that never says COMPLETED, and
     a stream flagged READY at the very end -- each must be caught, or the assertions above are ornament.

The pacing is the REAL `TourPacer`, driven by a clock this test owns and an odometry feed it synthesises:
the vehicle flies each leg at a constant speed, holds each corner for `hold_s`, and the arrival gate fires
on the actual horizontal distance.  Nothing is mocked except TIME and the vehicle.

usage: tools/test-cmd-stream.py [--world worlds/pillars.yaml] [--rate 100] [--hold-s 5.0]
"""
from __future__ import annotations

import argparse
import importlib.util
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from common import frames                            # noqa: E402
from planner.cmd_stream import (CmdStream, TourPacer, TRAJECTORY_STATUS_COMPLETED,   # noqa: E402
                                TRAJECTORY_STATUS_EMPTY, TRAJECTORY_STATUS_READY, FRAME_L)


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


def legs_of(corners, spacing):
    """The legs exactly as `planner/rectangle_source.py` builds them: OPEN polylines, one per edge."""
    out = []
    for i in range(len(corners)):
        a = np.asarray(corners[i], dtype=float)
        b = np.asarray(corners[(i + 1) % len(corners)], dtype=float)
        n = max(2, int(float(np.linalg.norm(b - a)) / max(spacing, 1e-3)) + 1)
        out.append(np.linspace(a, b, n))
    return out


def leg_len_w(leg):
    return float(np.sum(np.linalg.norm(np.diff(np.asarray(leg)[:, :2], axis=0), axis=1)))


def drive(corners, legs, hold_s, rate, speed, calib, arrive_m=0.10, leg_start_m=0.30,
          timeout_s=20.0, wrong=None):
    """Fly the tour on a clock this function owns, and return the samples the stream produced.

    The vehicle is a point that always moves TOWARD the corner the pacer is currently asking for, at
    `speed`, and stands still while the pacer dwells.  Its position is therefore a function of the
    pacer's own state -- which is what makes the arrival gate fire on the REAL horizontal distance
    (test 1/2/3 need that) and what makes this a simulation of the closed loop rather than of a table.

    `wrong` applies one deliberate defect to the producer, for the negative controls.
    """
    corners = np.asarray(corners, dtype=float)
    n_leg = len(legs)
    pacer = TourPacer(corners, n_leg, hold_s, arrive_m, leg_start_m, timeout_s, 0.0,
                      leg_len_w=lambda i: leg_len_w(legs[i]), empty_mark_s=2.0, leg_speed=speed)
    # THE LEGS IN L, computed exactly as the producer does: `R_LW @ (p_w - t_WL)`.  The command stream
    # takes these, not the W corners, so what it calls L is L by construction.
    R_LW = calib.R_WL.T
    legs_l = [(R_LW @ (np.asarray(leg, dtype=float) - calib.t_WL).T).T for leg in legs]
    stream = CmdStream(pacer, rate, legs_l, trajectory_id=1)

    if wrong == "rate":
        # a stream that DECLARES `rate` but emits at half of it.  `period` is what `due`/`advance` use,
        # so this is the honest way to lie about a rate, and the self-clocking check must catch it.
        stream.period = 2.0 / rate

    # THE SIMULATION'S TIME STEP IS A DIVISOR OF THE STREAM'S PERIOD, and that is not cosmetic: the
    # stream emits on a grid of `period`, so a clock that advances by an amount that does not divide the
    # period makes the EMISSION times land on a coarser grid (measured: a 2 ms step with a 10 ms period
    # produced 12 ms spacing, which the self-clocking assertion reports as a 2 ms error and would be a
    # false failure).  The pacer still gets steps of ~1 ms or finer, as a 50 Hz streaming loop would.
    k = max(1, int(math.ceil(rate / 1000.0)))         # >= 1000 Hz of pacer steps
    micro = (1.0 / rate) / k
    t = 0.0
    max_t = 400.0
    samples = []                          # (t, CmdSample, leg_index, dwelling, done)
    pb = corners[0][:2].copy()
    dist_left = float(np.linalg.norm(corners[1 % len(corners)][:2] - pb))
    while t < max_t:
        if not pacer.done and not pacer.dwelling:
            target = corners[(pacer.i_leg + 1) % len(corners)][:2]
            step = speed * micro
            if dist_left <= step:
                pb = target.copy()
            else:
                pb = pb + (target - pb) / dist_left * step
                dist_left -= step
        # WHILE DWELLING the vehicle stands still: `pb` keeps whatever it arrived with, because the
        # position loop really does stop the aircraft 5-30 mm short of its command (C1 measured 0.031 m)
        # and the command stream must keep asking for the CORNER while that happens.  That is what makes
        # the <= 1 mm dwell assertion mean something.
        b = pacer.dwelling
        pacer.odom_n = int(t * 200) + 1
        pacer.odom_wall = t
        # THE FRAMES, explicitly, because getting them wrong is what this whole refactor is about:
        #   * `pacer.step` takes the vehicle in W -- that is the frame the legs and the arrival gate are
        #     in (`here_w()` in the producer), so the driver hands it the raw world point;
        #   * the LIO sample the producer actually receives is that point's L image, which is what the
        #     driver would have to write if it were a sensor.  It is computed for the record only.
        p_w = np.array([pb[0], pb[1], corners[0][2]])
        lio_l = (calib.R_WL.T @ (p_w - calib.t_WL))       # == R_LW @ (p_w - t_WL), the producer's W->L
        if float(np.linalg.norm((calib.R_WL @ lio_l + calib.t_WL) - p_w)) >= 1e-12:
            raise AssertionError("the driver's own W->L->W round trip is not exact")
        pacer.step(p_w, t)
        if b and not pacer.dwelling:
            # the dwell is over: the next leg starts at the corner the vehicle is STANDING ON, not at the
            # point it happened to stop on.  That is the producer's own rule (a leg's clock arms within
            # `--leg-start-m` of the leg's START corner), and resetting here keeps the arrival error from
            # accumulating across four corners into a miss.
            pb = corners[pacer.i_leg][:2].copy()
            dist_left = float(np.linalg.norm(corners[(pacer.i_leg + 1) % len(corners)][:2] - pb))
        if stream.next_t is None:
            stream.next_t = t
        if stream.due(t):
            s = stream.sample()
            if wrong == "dwell-offset" and pacer.dwelling:
                # the dwell is reported 5 mm off the corner -- the 1 mm assertion must catch it
                s.p = np.asarray(s.p, dtype=float) + np.array([0.005, 0.0, 0.0])
            samples.append((t, s, pacer.i_leg, pacer.dwelling, pacer.done))
            stream.advance(t)
        t += micro
        # Stop `--empty-mark-s` after the COMPLETED statement, as the producer's loop does (the command
        # stream keeps publishing for that window so a consumer that is still settling cannot miss it,
        # and the message is latched, so a later joiner still finds it).  The test must not read the
        # silence after that window, and the break has to be tested on the TIME, not on
        # `t - samples[-1][0]`: the pacer stops stepping once it is done, so the sample times stop
        # advancing with the clock and the difference never grows -- MEASURED: that version of this loop
        # ran to its 400 s cap and reported a 360 s "done" segment, which is a property of the test and
        # not of the stream.
        if pacer.done and t >= pacer.t_done + 2.0:
            break
    return samples, pacer, stream


def seg_distance(p, leg):
    """Horizontal distance from `p` to the leg's polyline (the published geometry)."""
    pts = np.asarray(leg, dtype=float)[:, :2]
    p = np.asarray(p, dtype=float)[:2]
    best = float("inf")
    for i in range(len(pts) - 1):
        a, b = pts[i], pts[i + 1]
        d = b - a
        L2 = float(d @ d)
        if L2 < 1e-18:
            best = min(best, float(np.linalg.norm(p - a)))
            continue
        u = max(0.0, min(1.0, float((p - a) @ d) / L2))
        best = min(best, float(np.linalg.norm(p - (a + u * d))))
    return best


def check(name, got, want, tol):
    good = abs(float(got) - float(want)) <= tol
    print("   %-72s %s" % (name, "ok" if good else "FAIL got=%.6f want=%.6f (tol %.6g)"
                           % (got, want, tol)))
    return good


def group_samples(samples):
    """The samples grouped into maximal runs of one (leg, dwelling, done) state.

    A "segment" is what the stream said without changing its mind: a flight along one leg, a dwell on one
    corner, or the COMPLETED statement at the end.  Every assertion below is about a segment, because
    "the dwell lasted 5 s" is only a statement about a segment.
    """
    groups = []
    for row in samples:
        key = (row[2], row[3], row[4])
        if groups and groups[-1][0] == key:
            groups[-1][1].append(row)
        else:
            groups.append((key, [row]))
    return groups


def run_checks(samples, corners, legs, hold_s, rate, calib, legs_l=None, quiet=False):
    """Every assertion of the contract, expressed once so the controls can reuse it."""
    problems = []
    if legs_l is None:                    # default: convert here, so a control cannot forget to
        R_LW = calib.R_WL.T
        legs_l = [(R_LW @ (np.asarray(leg, dtype=float) - calib.t_WL).T).T for leg in legs]

    def bad(msg):
        problems.append(msg)
        if not quiet:
            print("   FAIL %s" % msg)

    def ok(cond, msg):
        if not cond:
            bad(msg)
        return cond

    # ---- 1. on the commanded path, within 1 mm -----------------------------------------------------
    # The stream is in L and the legs are in W, so the comparison goes through the calibration once --
    # and test 2 below is what proves that conversion is the right one.  Comparing the raw numbers would
    # be comparing two frames, which is the mistake this refactor exists to remove.
    def to_w(p_l):
        return calib.R_WL @ np.asarray(p_l, dtype=float) + calib.t_WL

    worst = 0.0
    for t, s, i_leg, dwell, done in samples:
        p_w = to_w(s.p)
        if dwell or done:
            d = float(np.linalg.norm(p_w[:2] - corners[(i_leg + 1) % len(corners)][:2]))
        else:
            d = seg_distance(p_w, legs[i_leg])
        worst = max(worst, d)
    ok(worst <= 1e-3, "the streamed positions are not on the commanded path: worst %.6f m > 1 mm"
       % worst)

    # ---- 2. in L, and consistent with the W corner that is actually commanded ----------------------
    worst_w = 0.0
    for t, s, i_leg, dwell, done in samples:
        if dwell or done:
            want_w = corners[(i_leg + 1) % len(corners)]
            got_w = to_w(s.p)
            worst_w = max(worst_w, float(np.linalg.norm(got_w - want_w)))
    ok(worst_w <= 1e-9, "R_WL @ p_L + t_WL does not reproduce the commanded W corner: worst %.3e m"
       % worst_w)

    # ---- 3. the dwell is content ------------------------------------------------------------------
    # group consecutive samples by (leg, dwell, done)
    groups = group_samples(samples)
    dwells = [(k, g) for k, g in groups if k[1] and not k[2]]
    ok(len(dwells) == len(corners), "expected %d corner dwells, got %d" % (len(corners), len(dwells)))
    for k, g in dwells:
        i_leg = k[0]
        # THE DWELL IS MEASURED IN SAMPLES, not wall time: the last sample of the segment is emitted one
        # period BEFORE the dwell ends (the dwell ends on the pacer's next micro-step), so the sample
        # span is `hold_s - period` at worst.  `tools/dwell-audit.py` measures the flown dwell the same
        # way -- from samples, at a radius -- which is why the tolerance is one sample period and not 0.
        span = g[-1][0] - g[0][0]
        corner = corners[(i_leg + 1) % len(corners)]
        off = max(float(np.linalg.norm(to_w(r[1].p)[:2] - corner[:2])) for r in g)
        vmax = max(float(np.linalg.norm(r[1].v)) for r in g)
        amax = max(float(np.linalg.norm(r[1].a)) for r in g)
        ok(span >= hold_s - stream_period(rate) - 1e-9,
           "dwell at corner %d lasted %.4f s < hold_s %.4f s (one sample period %.4f s is allowed)"
           % ((i_leg + 1) % len(corners), span, hold_s, stream_period(rate)))
        ok(off <= 1e-3, "dwell at corner %d is %.6f m off the corner (> 1 mm)"
           % ((i_leg + 1) % len(corners), off))
        ok(vmax == 0.0, "dwell at corner %d carries velocity %.6g, not 0"
           % ((i_leg + 1) % len(corners), vmax))
        ok(amax == 0.0, "dwell at corner %d carries acceleration %.6g, not 0"
           % ((i_leg + 1) % len(corners), amax))
    # ---- 2b. the yaw is the LEG's tangent in L (the Path's per-point convention, one number) --------
    # The stream's yaw while FLYING leg i is leg i's own tangent; while DWELLING on the corner that leg i
    # ends on it is leg i's tangent too (the heading it arrived on and holds).  Both are read from the L
    # leg geometry, which is the same source the Path's per-point yaw comes from.
    for k, g in groups:
        i_leg = k[0]
        want = CmdStream._leg_yaw(legs_l[i_leg % len(legs_l)])
        got = g[0][1].yaw
        ok(abs(math.atan2(math.sin(got - want), math.cos(got - want))) < 1e-9,
           "yaw of leg %d is %.6f rad, the leg's own tangent in L is %.6f" % (i_leg, got, want))

    # ---- 4. self-clocked: the samples of a segment are one period apart ----------------------------
    dev = 0.0
    n_pairs = 0
    for k, g in groups:
        for a, b in zip(g, g[1:]):
            dev = max(dev, abs((b[0] - a[0]) - stream_period(rate)))
            n_pairs += 1
    ok(n_pairs > 0 and dev <= 1e-6, "the stream is not self-clocked at %g Hz: worst spacing error "
       "%.3e s over %d pairs" % (rate, dev, n_pairs))
    total = samples[-1][0] - samples[0][0]
    measured = (len(samples) - 1) / total if total > 0 else float("inf")
    ok(abs(measured - rate) <= 0.02 * rate,
       "the stream's own rate is %.3f Hz, declared %g Hz (2%% band)" % (measured, rate))

    # ---- 5. the last message carries COMPLETED, and EMPTY precedes the first leg -------------------
    ok(samples[-1][1].completed, "the last sample's trajectory_flag is %d, not COMPLETED(%d)"
       % (samples[-1][1].trajectory_flag, TRAJECTORY_STATUS_COMPLETED))
    ok(samples[0][1].trajectory_flag == TRAJECTORY_STATUS_EMPTY,
       "the FIRST sample's flag is %d, not EMPTY(%d) -- before the tour is armed there is nothing to fly"
       % (samples[0][1].trajectory_flag, TRAJECTORY_STATUS_EMPTY))
    ok(all(s[1].trajectory_id != 0 for s in samples),
       "a sample carries trajectory_id 0, which the message documents as 'no trajectory yet'")
    ok(all(s[1].frame_id == FRAME_L for s in samples),
       "a sample's frame_id is not %r" % FRAME_L)
    ok(all(list(s[1].kx) == [0.0] * 3 and list(s[1].kv) == [0.0] * 3 for s in samples),
       "kx/kv are not zero: the message has no gain fields on the wire (msg 84) and ego also sends zeros")
    return problems


def stream_period(rate):
    return 1.0 / rate


def self_test(args, corners, legs, calibs):
    """The negative controls: mutate the producer (never the test) and require a failure."""
    ok = True
    controls = [
        ("dwell position moved 5 mm off the corner", "dwell-offset", "1 mm"),
        ("a rate declared at 100 Hz but emitted at 50 Hz", "rate", "self-clocked"),
        ("a stream that never says COMPLETED", "nocomplete", "COMPLETED"),
    ]
    for label, mode, needle in controls:
        print()
        print("NEGATIVE CONTROL: %s" % label)
        if mode == "nocomplete":
            samples, pacer, stream = drive(corners, legs, args.hold_s, args.rate, args.speed,
                                           calibs[0])
            # keep only the samples before the tour is declared done: the stream then ends READY, which
            # is exactly "the producer stops talking without saying it finished"
            samples = [s for s in samples if not s[4]]
            if not samples:
                raise SystemExit("negative control 'nocomplete' produced no samples -- control is stale")
        else:
            samples, pacer, stream = drive(corners, legs, args.hold_s, args.rate, args.speed,
                                           calibs[0], wrong=mode)
        problems = run_checks(samples, corners, legs, args.hold_s, args.rate, calibs[0], quiet=True)
        hit = [p for p in problems if needle in p]
        print("   the mutated producer produced %d sample(s) and the contract test reported %d "
              "problem(s)" % (len(samples), len(problems)))
        print("   verdict: %s" % ("FAILS as required" if hit else "DID NOT FAIL -- the control is dead"))
        for p in (hit or problems)[:2]:
            print("      %s" % p)
        if not hit:
            ok = False
    return ok


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default=os.path.join(ROOT, "worlds", "pillars.yaml"))
    ap.add_argument("--spacing", type=float, default=0.05)
    ap.add_argument("--rate", type=float, default=100.0, help="the stream's declared rate, Hz")
    ap.add_argument("--hold-s", type=float, default=5.0)
    ap.add_argument("--speed", type=float, default=0.30)
    ap.add_argument("--self-test", dest="self_test", action="store_true", default=True,
                    help="run the negative controls (default: on -- a control that is not run by "
                         "default is a control that stops being run)")
    ap.add_argument("--no-self-test", dest="self_test", action="store_false")
    args = ap.parse_args()

    world = read_world(args.world)
    corners = corners_of(world)
    legs = legs_of(corners, args.spacing)
    print("world %s: %d corners, %d open legs" % (os.path.basename(args.world), len(corners),
                                                  len(legs)))

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
        samples, pacer, stream = drive(corners, legs, args.hold_s, args.rate, args.speed, calib)
        # a breakdown, so the shape of the tour is visible and not just its verdict
        for key, g in group_samples(samples):
            kind = "COMPLETED" if key[2] else ("dwell  " if key[1] else "fly    ")
            print("   leg %d %s %6d samples  t=%8.3f..%8.3f  span %6.3f s"
                  % (key[0], kind, len(g), g[0][0], g[-1][0], g[-1][0] - g[0][0]))
        print("   %s" % stream.summary())
        print("   pacer: done=%s, legs flown=%d" % (pacer.done, pacer.i_leg + 1))
        if not pacer.done:
            problems.append("calibration %d: the tour never completed (pacer.i_leg=%d)"
                            % (i, pacer.i_leg))
        problems += ["calib %d: %s" % (i, p) for p in
                     run_checks(samples, corners, legs, args.hold_s, args.rate, calib)]

    ok = not problems
    if args.self_test:
        ok = self_test(args, corners, legs, cals) and ok

    print()
    if problems:
        print("CMD-STREAM TEST FAILED (%d problem(s)):" % len(problems))
        for p in problems[:12]:
            print("   - %s" % p)
    print("CMD-STREAM TEST %s" % ("PASSED" if ok else "FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
