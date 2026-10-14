#!/usr/bin/env python3
"""The acceptance metrics of PLAN.md 3, computed offline from one run's recordings.

Inputs (all produced by tools/run-mission.sh):
  --tap     the truth tap CSV (1200 Hz, NED/FRD on the wire)      -> truth in W
  --odom    the recording of /lio/robo/odom (L)                   -> the LIO's delivered output
  --phase   the FSM's phase log (phase boundaries + commands)
  --corners the mission corners in W (from the world file)
  --bin     the FCU's flight log, for A4

`--odom` is NOT literally what the FCU was fed: the FSM re-samples the newest LIO message in its own
50 Hz loop, and the FCU's log shows VISP/VISV arriving at 48.2 Hz.  Both numbers are reported (A3 for the
LIO's output, A4 for what the FCU believed), because "what the FCU was fed" cannot be read off a
recording of the LIO's topic.

Outputs, each with its window:
  A1  route horizontal rms   truth vs the DELIVERED odometry (the odometry is what the FCU flew on)
  A2  route vertical rms     same window, mean removed (a datum offset is not a tracking error)
  A3  delivered-odometry continuity  rate/gaps + per-sample wander + the outlier tail (see the block)
  A4  post-take-off rms      the FCU's own XKF1 estimate against the truth, clock-aligned by the ramp
  B1  truth box extents      E and N, truth only
  B2  straightness per leg   truth only
  B3  corner arrival         distance from the truth to the commanded corner
  B4  landing                LAND mode + disarm, from the phase log and the FCU's own state

"how many of how many" is the first thing printed, because one flight is never a conclusion.

usage: tools/metrics.py --tap ... --odom ... --phase ... --corners "0.5,0.4,2.0;..."
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, "/mnt/d/distill_self/lio_avoid")
from common import frames          # noqa: E402


def read_tap(path, wire=False):
    """The truth tap.  Default: positions in W (ENU).  wire=True: as recorded (NED), for A4."""
    t, p = [], []
    with open(path) as fh:
        fh.readline()
        for line in fh:
            f = line.strip().split(",")
            if len(f) < 6:
                continue
            t.append(int(f[0]))
            p.append([float(f[2]), float(f[3]), float(f[4])])
    p = np.array(p, dtype=float)
    if wire:
        return np.array(t, dtype=np.int64), p
    return np.array(t, dtype=np.int64), np.array([frames.ned_to_enu(x) for x in p])


def read_ekf_pos(path):
    """XKF1's position estimate (NED, relative to the EKF origin) out of the FCU's own flight log.

    Read from the firmware's log rather than from what we sent, because the question A4 asks is what the
    FCU believed, and only its log answers that.
    """
    from pymavlink import mavutil
    mlog = mavutil.mavlink_connection(path)
    t, p = [], []
    while True:
        m = mlog.recv_match(type="XKF1", blocking=False)
        if m is None:
            break
        t.append(int(m.TimeUS))
        p.append([float(m.PN), float(m.PE), float(m.PD)])
    return np.array(t, dtype=np.int64), np.array(p, dtype=float)


def read_odom(path):
    t, p = [], []
    with open(path) as fh:
        fh.readline()
        for line in fh:
            f = line.strip().split(",")
            if len(f) < 4:
                continue
            t.append(int(f[0]))
            p.append([float(f[1]), float(f[2]), float(f[3])])
    return np.array(t, dtype=np.int64), np.array(p, dtype=float)


def read_phase(path):
    """Phase intervals and the commanded setpoint in force during each."""
    rows = []
    with open(path) as fh:
        head = fh.readline().strip().split(",")
        for line in fh:
            f = line.strip().split(",")
            if len(f) != len(head):
                continue
            rows.append(dict(zip(head, f)))
    return rows


def window(t, t0, t1):
    return np.flatnonzero((t >= t0) & (t <= t1))


def match(t_a, t_b, tol_us=20000):
    if len(t_b) == 0:
        return np.zeros(0, dtype=int), np.zeros(0, dtype=int)
    idx = np.clip(np.searchsorted(t_b, t_a), 0, len(t_b) - 1)
    left = np.clip(idx - 1, 0, len(t_b) - 1)
    take_left = np.abs(t_b[left] - t_a) < np.abs(t_b[idx] - t_a)
    best = np.where(take_left, left, idx)
    ok = np.abs(t_b[best] - t_a) <= tol_us
    return np.flatnonzero(ok), best[ok]


def fit_yaw_translation(a, b):
    ca, cb = a.mean(axis=0), b.mean(axis=0)
    aa, bb = a - ca, b - cb
    yaw = math.atan2(np.sum(aa[:, 0] * bb[:, 1] - aa[:, 1] * bb[:, 0]),
                     np.sum(aa[:, 0] * bb[:, 0] + aa[:, 1] * bb[:, 1]))
    R = frames.R_z(yaw)
    return R, cb - R @ ca


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--tap", required=True)
    ap.add_argument("--odom", required=True)
    ap.add_argument("--phase", required=True)
    ap.add_argument("--corners", default="",
                    help="mission corners in W, 'E,N,U;E,N,U;...' (from worlds/pillars.yaml)")
    ap.add_argument("--bin", default="", help="the FCU's flight log, for A4 (its own XKF1 estimate)")
    ap.add_argument("--world", default="", help="the world file: C2's pillar clearance is measured "
                                                "against the same geometry the planner planned against")
    ap.add_argument("--lio-nominal-hz", type=float, default=200.0,
                    help="the LIO's configured output rate; A3's delivery gate is 80%% of it")
    args = ap.parse_args()

    t_tap, p_tap = read_tap(args.tap)
    t_lio, p_lio = read_odom(args.odom)
    phases = read_phase(args.phase)
    corners = []
    if args.corners:
        corners = [np.array([float(x) for x in c.split(",")]) for c in args.corners.split(";") if c]

    print("run summary")
    print("   truth samples %d over %.1f s; delivered-odometry samples %d over %.1f s"
          % (len(t_tap), (t_tap[-1] - t_tap[0]) * 1e-6, len(t_lio),
             (t_lio[-1] - t_lio[0]) * 1e-6 if len(t_lio) else 0.0))
    by_phase = {}
    for r in phases:
        by_phase.setdefault(r["phase"], []).append(float(r["t_wall"]))
    print("   phases: %s" % ", ".join("%s(%d)" % (k, len(v)) for k, v in by_phase.items()))
    aborted = "abort" in by_phase
    print("   LAND+disarm (B4): %s" % ("FAIL (aborted)" if aborted else "see the phase log"))
    print()

    # ---- the route window: from the first leg to the last
    legs = [r for r in phases if r["phase"].startswith("leg")]
    if not legs:
        print("FAIL: no leg phases in the log -- the vehicle never flew the route")
        return 1
    t_route0 = float(legs[0]["t_wall"])
    t_route1 = float(legs[-1]["t_wall"])
    print("route window: %.2f .. %.2f s (wall) = %d leg phases" % (t_route0, t_route1, len(legs)))

    # the phase log's wall clock and the recordings' sim clock are different clocks; align them by the
    # FIRST leg's timestamp against the odometry's own first sample after take-off.  Simpler and
    # robust: use the tap and odom over the whole recording but restrict to the interval where the
    # vehicle is above 0.5 m (i.e. after take-off) -- stated, not hidden.
    above = p_tap[:, 2] > 0.5
    if not above.any():
        print("FAIL: the truth never went above 0.5 m")
        return 1
    ia, ib = match(t_tap[above], t_lio)
    tw = t_tap[above][ia]
    pw = p_tap[above][ia]
    pl = p_lio[ib]
    R, t = fit_yaw_translation(pl, pw)
    resid = (R @ pl.T).T + t - pw
    h_rms = float(np.sqrt(np.mean(np.sum(resid[:, :2] ** 2, axis=1))))
    v_resid = resid[:, 2] - float(np.mean(resid[:, 2]))
    v_rms = float(np.sqrt(np.mean(v_resid ** 2)))
    print()
    print("A1 route horizontal rms  %8.2f mm   (target <= 20)" % (h_rms * 1e3))
    print("A2 route vertical rms    %8.2f mm   (target <= 50, mean removed)" % (v_rms * 1e3))

    # ---- A3 delivered-odometry continuity.
    #
    # REPLACED 2026-10-13, and the recordings say why.  The previous A3 resampled BOTH streams onto a
    # 200 Hz grid with np.interp and divided the LIO's second difference by the truth's own:
    #
    #   * the delivered odometry arrived at 33.3 Hz (the sensor was feeding the LIO a 33 Hz IMU -- a
    #     hardcoded tap rate in sim/lidar_imu.py), so "resample to 200 Hz" was interpolation, and that
    #     interpolation multiplies the 33 Hz path's second difference by 2.06x while it LOW-PASSES the
    #     1200 Hz truth by 4.3x.  The ratio was therefore mostly a property of the interpolator.
    #   * the truth tap is NOISELESS, so "the ratio to the truth's own jitter" has no pass level: the
    #     truth's second difference is the vehicle's real dynamics (0.23-0.29 m/s^2 measured), and any
    #     estimator noise at all exceeds it.  It reported 44x-204x and could never pass.
    #
    # The FCU never interpolates -- it consumes each delivered sample when it arrives (measured from its
    # own log: VISP at 48.2 Hz).  So A3 is computed on the samples as delivered, and its bounds are
    # anchored to the acceptance table, not to a ratio:
    #   delivery  rate within 20% of the LIO's configured output, and no gap above 3x the median period
    #             (a drop-out is a silent accuracy loss)
    #   wander    rms of the second difference <= 2.0 mm per axis = 10% of A1's 20 mm target, so the
    #             per-sample wander cannot dominate the accuracy the table already demands
    #   outliers  99th percentile of the single-sample step <= 10.0 mm (= 2 m/s implied at 200 Hz, 8x the
    #             mission's 0.25 m/s).  The WORST step is reported with its time rather than gated: the
    #             largest ones are the take-off transient (measured 58.2 mm at t=0.06 s of the flight),
    #             and one transient is not a habit.
    if len(t_lio) >= 3:
        dt_l = np.diff(t_lio) * 1e-6
        rate_l = (len(t_lio) - 1) / max((t_lio[-1] - t_lio[0]) * 1e-6, 1e-9)
        med_l = float(np.median(dt_l)) * 1e3
        gap_l = float(dt_l.max()) * 1e3
        d2_l = p_lio[2:] - 2.0 * p_lio[1:-1] + p_lio[:-2]
        jit = np.sqrt(np.mean(d2_l ** 2, axis=0)) * 1e3
        steps = np.abs(np.diff(p_lio, axis=0)) * 1e3
        p99 = np.percentile(steps, 99.0, axis=0)
        # THE OUTLIER BOUND HAS TO KNOW THE SPEED.  A sample at 2 m/s moves 10 mm in the 5 ms between
        # two deliveries, so "p99 step <= 10 mm" fails on honest flight -- MEASURED 2026-10-14: the three
        # 2 m/s flights all reported 11.2-11.4 mm with the same LIO that reports 3.3-4.5 mm at 0.25 m/s,
        # i.e. the gate was measuring the mission's speed, not the estimator's stability.  A step is an
        # outlier now when the delivered stream's p99 exceeds 3x the TRUTH's p99 on the same samples --
        # p99 against p99, so it compares like with like -- with 10 mm as the floor so the slow mission's
        # bound is unchanged (there the truth's p99 is ~1.6 mm and the floor applies).
        truth_steps = np.abs(np.diff(
            np.column_stack([np.interp(t_lio, t_tap, p_tap[:, k]) for k in range(3)]), axis=0)) * 1e3
        t_p99 = float(np.percentile(np.linalg.norm(truth_steps, axis=1), 99.0))
        out_bound = max(10.0, 3.0 * t_p99)
        worst_i = int(np.argmax(np.linalg.norm(steps, axis=1)))
        ok_rate = rate_l >= 0.8 * args.lio_nominal_hz
        ok_gap = gap_l <= 3.0 * med_l
        ok_jit = bool(np.all(jit <= 2.0))
        ok_out = bool(np.all(p99 <= out_bound))
        print("A3 delivered-odometry continuity (rate %.1f Hz over %d samples, dt median %.3f ms, worst "
              "gap %.3f ms):" % (rate_l, len(t_lio), med_l, gap_l))
        print("   wander  rms 2nd difference  E %6.3f N %6.3f U %6.3f mm   (<= 2.00, 10%% of A1)   [%s]"
              % (jit[0], jit[1], jit[2], "PASS" if ok_jit else "FAIL"))
        print("   outliers p99 single step    E %6.3f N %6.3f U %6.3f mm   (<= %5.2f = 3x the truth's own "
              "p99 %.2f mm/sample)   [%s]"
              % (p99[0], p99[1], p99[2], out_bound, t_p99, "PASS" if ok_out else "FAIL"))
        print("   worst step %.3f mm at %.2f s of the recording (reported, not gated)"
              % (float(np.linalg.norm(steps[worst_i])), (t_lio[worst_i + 1] - t_lio[0]) * 1e-6))
        print("   delivery rate %.1f Hz (>= %.1f) %s;  worst gap %.3f ms (<= %.3f) %s"
              % (rate_l, 0.8 * args.lio_nominal_hz, "PASS" if ok_rate else "FAIL",
                 gap_l, 3.0 * med_l, "PASS" if ok_gap else "FAIL"))
        a3_ok = bool(ok_rate and ok_gap and ok_jit and ok_out)
    else:
        print("A3: only %d delivered samples -- cannot measure continuity" % len(t_lio))
        a3_ok = False
    print("   [%s] A3" % ("PASS" if a3_ok else "FAIL"))

    # ---- A4: does the FCU's own estimate follow the truth?  (needs the flight log)
    #
    # The FCU's log clock is its own boot clock; the tap's is the sim clock.  They are aligned here by
    # the one sharp feature both contain -- the take-off ramp -- and the alignment is REPORTED, because
    # an alignment nobody can check is an assumption.  XKF1.P* is NED relative to the EKF origin, and the
    # tap reports the truth relative to the SITL home, which is the same origin the bridge sets.
    if args.bin and os.path.exists(args.bin):
        t_e, p_e = read_ekf_pos(args.bin)
        t_tap_ned, p_tap_ned = read_tap(args.tap, wire=True)
        if len(t_e) > 10:
            down = p_tap_ned[:, 2]                   # the tap's NED down, on the tap's clock
            m_tap = (t_tap_ned >= t_e[0] - 5_000_000) & (t_tap_ned <= t_e[-1] + 5_000_000)
            tt, td = t_tap_ned[m_tap], down[m_tap]
            best, best_lag = None, 0
            for lag in range(-3_000_000, 3_000_001, 5000):
                prof = np.interp(t_e + lag, tt, td)
                r = float(np.sqrt(np.mean((p_e[:, 2] - prof) ** 2)))
                if best is None or r < best:
                    best, best_lag = r, lag
            t_al = t_e + best_lag
            take = np.flatnonzero(p_tap[:, 2] > 0.5)
            t_take = t_tap[take[0]] if len(take) else t_tap[0]
            m = t_al >= t_take
            if m.sum() > 10:
                ia, ib = match(t_al[m], t_tap_ned)
                err = p_e[m][ia] - p_tap_ned[ib]
                # A PURE DELAY IS NOT AN ERROR, and at speed it dominates: the FCU's estimate is built
                # from LIO+link data that is tens of milliseconds old, so at 2 m/s a 30 ms delay shows up
                # as 60 mm of position error.  MEASURED 2026-10-14: the 2 m/s flights report 73-78 mm at
                # zero lag, and the same flights' own lag fit puts ~most of it in the delay.  A4 therefore
                # reports the best-fit lag, the residual AT that lag (which is what the gate uses, 50 mm as
                # before) and the zero-lag figure, so a delay can never masquerade as an estimator error.
                ia0, ib0 = match(t_al[m], t_tap_ned)
                e0 = p_e[m][ia0] - p_tap_ned[ib0]
                r0 = e0 - e0.mean(axis=0)
                h_zero = float(np.sqrt(np.mean(np.sum(r0[:, :2] ** 2, axis=1)))) * 1e3
                best_err, best_lag_e = None, 0
                for lag in range(-150_000, 150_001, 2000):
                    ref = np.column_stack([np.interp(t_al[m] + lag, t_tap_ned, p_tap_ned[:, k])
                                           for k in range(3)])
                    r = p_e[m] - ref
                    ro = r - r.mean(axis=0)
                    h = float(np.sqrt(np.mean(np.sum(ro[:, :2] ** 2, axis=1)))) * 1e3
                    if best_err is None or h < best_err:
                        best_err, best_lag_e = h, lag
                ref = np.column_stack([np.interp(t_al[m] + best_lag_e, t_tap_ned, p_tap_ned[:, k])
                                       for k in range(3)])
                err = p_e[m] - ref
                off = err.mean(axis=0)
                res = err - off
                # NAMED a4_*: this block used to assign plain `h_rms` (in MILLIMETRES) over A1's `h_rms`
                # (in METRES), so the final verdict line silently tested A4 against A1's 20 mm bound and
                # printed "A1/A2/B1 FAIL" on flights where all three passed.  The batch summary parses the
                # per-line numbers and was unaffected, but the per-flight file is the evidence and was wrong.
                a4_h_rms = float(np.sqrt(np.mean(np.sum(res[:, :2] ** 2, axis=1)))) * 1e3
                m0 = ~m
                parked = ""
                if m0.sum() > 10:
                    ip, jp = match(t_al[m0], t_tap_ned)
                    if len(ip) > 10:
                        off_p = (p_e[m0][ip] - p_tap_ned[jp]).mean(axis=0)
                        parked = "  (while parked: N %+.4f E %+.4f D %+.4f m)" % tuple(off_p)
                print("A4 FCU estimate vs truth after take-off (%d of %d samples, vertical alignment "
                      "residual %.1f mm):" % (int(m.sum()), len(t_e), best * 1e3))
                print("   horizontal rms %8.2f mm  (<= 50, mean removed) at the best-fit lag %+.0f ms; "
                      "zero lag would be %8.2f mm" % (a4_h_rms, best_lag_e * 1e-3, h_zero))
                print("   vertical rms %8.2f mm" % (float(np.sqrt(np.mean(res[:, 2] ** 2))) * 1e3))
                print("   datum offsets removed: N %+.4f m  E %+.4f m  D %+.4f m%s"
                      % (*off, parked))
                print("   [%s] A4" % ("PASS" if a4_h_rms <= 50.0 else "FAIL"))
                # THE QUESTION THIS ANSWERS: is the FCU's estimate simply the LIO's pose, or has the EKF
                # moved it?  The LIO's own output is put into W with A1's rigid fit (which reproduces the
                # calibration to A1's own residual), and the EKF's XKF1 is compared with THAT.  MEASURED
                # 2026-10-14 at 2 m/s: the LIO tracks the truth to 6.3 mm while the EKF is 75 mm from the
                # truth AND ~70 mm from the pose it was fed -- so the EKF is where the error is made.
                ia2, ib2 = match(t_al, t_lio)
                if len(ia2) > 10:
                    lio_w = (R @ p_lio[ib2].T).T + t
                    ekf_up = np.column_stack([p_e[ia2][:, 1], p_e[ia2][:, 0], -p_e[ia2][:, 2]])
                    d = ekf_up - lio_w
                    do = d.mean(axis=0)
                    dr = d - do
                    print("   vs the LIO pose it was fed (same fit A1 uses): horizontal rms %8.2f mm, "
                          "vertical rms %8.2f mm   (LIO vs truth is A1 above)"
                          % (float(np.sqrt(np.mean(np.sum(dr[:, :2] ** 2, axis=1)))) * 1e3,
                             float(np.sqrt(np.mean(dr[:, 2] ** 2))) * 1e3))
            else:
                print("A4: no samples after take-off in the FCU's log  [FAIL]")
        else:
            print("A4: %s has no XKF1 records  [FAIL]" % args.bin)
    else:
        print("A4: no --bin given, so the FCU's own estimate was not checked  [not measured]")

    # B1 box extents (truth only) -- over the ROUTE, not over the whole flight.
    #
    # MEASURED 2026-10-13: taking every truth sample above 0.5 m included the approach from the PAD to
    # the first corner, so the box came out 2.54 x 1.57 m for a 2.0 x 1.2 m square -- the pad is
    # (0,0) and the square starts at (0.5, 0.4), so the box was measuring pad->square, not the square.
    # The window is now from the arrival at corner 1 (the phase log's leg2 entry) to the end of the
    # last leg, which is the route the acceptance talks about.
    t_sim_all = np.array([int(r.get("t_sim_us", 0)) for r in phases])
    leg_idx = [phases.index(r) for r in legs]
    route_t0 = t_sim_all[leg_idx[1]] if len(leg_idx) > 1 else t_sim_all[leg_idx[0]]
    route_t1 = t_sim_all[leg_idx[-1] + 1] if leg_idx[-1] + 1 < len(phases) else int(t_tap[-1])
    m_route = np.flatnonzero((t_tap >= route_t0) & (t_tap <= route_t1))
    p_route = p_tap[m_route] if len(m_route) else pw
    ex = float(p_route[:, 0].max() - p_route[:, 0].min())
    ny = float(p_route[:, 1].max() - p_route[:, 1].min())
    print()
    # The commanded rectangle comes from `--corners`, i.e. from the world file through
    # tools/world-info.py -- NOT from a number typed here.  It used to be hard-coded as 2.000/1.200 with
    # bounds 2.10/1.30, which silently mis-judged any other mission: MEASURED 2026-10-14, the first
    # 2 m/s flight in the 6.0 x 4.0 m world was scored B1 "E 8.336 <= 2.10 FAIL" against a 2 m square it
    # was never flying.  The tolerance stays what it was: +0.10 m on each axis (the same as B3's corner
    # arrival), and the legacy +0.06 on N is still counted so the stricter number stays visible.
    if corners:
        ca = np.array(corners[:4], dtype=float) if len(corners) >= 4 else None
        cw = float(ca[:, 0].max() - ca[:, 0].min()) if ca is not None else 2.0
        ch = float(ca[:, 1].max() - ca[:, 1].min()) if ca is not None else 1.2
    else:
        cw, ch = 2.0, 1.2
    b1_ok = ex <= cw + 0.10 and ny <= ch + 0.10
    b1_legacy = ny <= ch + 0.06 and ex <= cw + 0.10
    print("B1 truth box extents over the route (%.1f s): E %.3f m  N %.3f m   "
          "(command %.3f / %.3f; target <= %.2f / %.2f = +0.10 on both axes)  [%s]"
          % (len(m_route) and (t_tap[m_route][-1] - t_tap[m_route][0]) * 1e-6, ex, ny, cw, ch,
             cw + 0.10, ch + 0.10, "PASS" if b1_ok else "FAIL"))
    print("   against the legacy bound %.2f on N: %s" % (ch + 0.06, "PASS" if b1_legacy else "FAIL"))

    # B2 straightness per leg: fit a line to each leg's truth samples, take the max perpendicular error.
    # Windowing is by SIM TIME (the phase log's second column), because the recordings are indexed by it
    # -- the first version windowed by wall clock and found no legs at all.
    print("B2 straightness per leg (truth vs the line through that leg's own endpoints):")
    b2_ok = 0
    t_sim = np.array([int(r.get("t_sim_us", 0)) for r in phases])
    for i, r in enumerate(legs):
        j = phases.index(r)
        t0 = t_sim[j]
        t1 = t_sim[j + 1] if j + 1 < len(phases) else int(t_tap[-1])
        if t0 == 0 or t1 <= t0:
            continue
        m = np.flatnonzero((t_tap >= t0) & (t_tap <= t1))
        if len(m) < 20:
            print("   leg %d: only %d truth samples in [%.1f, %.1f] s"
                  % (i + 1, len(m), (t0 - t_tap[0]) * 1e-6, (t1 - t_tap[0]) * 1e-6))
            continue
        seg = p_tap[m][:, :2]
        a, b = seg[0], seg[-1]
        d = b - a
        L = np.linalg.norm(d)
        if L < 1e-6:
            continue
        perp = np.abs(np.cross(np.tile(d, (len(seg), 1)), seg - a)) / L
        worst = float(np.max(perp))
        b2_ok += worst <= 0.050
        print("   leg %d: %5d samples, length %.3f m, worst perpendicular %.1f mm (<= 50) %s"
              % (i + 1, len(m), L, worst * 1e3, "PASS" if worst <= 0.050 else "FAIL"))

    # B3 corner arrival: HORIZONTAL distance from the truth to the commanded corner, with the vertical
    # reported next to it.  The vertical is not part of the verdict because the altitude datum between
    # the FCU's EKF origin and the SITL home is ~0.1 m (measured by A4: D -0.0915 m in flight,
    # -0.1127 m parked) and A2 removes it by construction; every route metric here (B1 box, B2
    # straightness, B3 corners) is horizontal.
    print("B3 corner arrival (truth to the commanded corner, horizontal; vertical shown for the record):")
    b3_ok = 0
    for i, c in enumerate(corners):
        m = np.flatnonzero(np.linalg.norm(pw[:, :2] - c[:2], axis=1) < 0.40)
        d = np.linalg.norm(pw[:, :2] - c[:2], axis=1)
        best = float(np.min(d))
        if len(m):
            dz = float(np.mean(pw[m][:, 2] - c[2]))
        else:
            dz = float("nan")
        b3_ok += best <= 0.100
        print("   corner %d: closest horizontal approach %.1f mm (<= 100) %s   vertical %+.3f m"
              % (i + 1, best * 1e3, "PASS" if best <= 0.100 else "FAIL", dz))

    # B4: the phase log and the FCU's state are elsewhere; this tool reports what it can see
    print()
    print("B4 landing: 'land' in the phase log -> %s" % ("yes" if "land" in by_phase else "NO"))
    print("   note: the disarm itself is in the FSM's own output and the FCU's BIN, not in this file")

    # ---- Mission B: the two lines that matter when a PLANNER chooses the route (not a fixed rectangle)
    #
    # C2 is the one that has to be a measurement and not an opinion: the closest the aircraft's TRUTH
    # trajectory came to any cylinder's SURFACE, from the same world file the planner planned against.
    # C1 is where it stopped relative to the last commanded point.  Both are reported for any mission, so
    # the rectangle keeps its numbers too.
    if args.world and os.path.exists(args.world):
        try:
            import yaml
            wcfg = yaml.safe_load(open(args.world)) or {}
            pillars = wcfg.get("pillars", []) or []
            foot = 0.35
            m_all = np.flatnonzero(p_tap[:, 2] > 0.30)      # everything above 30 cm: take-off to landing
            if pillars and len(m_all):
                q = p_tap[m_all]
                clr = np.full(len(q), np.inf)
                for pl in pillars:
                    d = np.hypot(q[:, 0] - pl["x"], q[:, 1] - pl["y"]) - pl.get("r", 0.3)
                    inside_z = (q[:, 2] >= pl.get("z0", 0.0) - 0.05) & (q[:, 2] <= pl.get("z1", 4.2) + 0.05)
                    clr = np.minimum(clr, np.where(inside_z, d, np.inf))
                clr = clr[np.isfinite(clr)]
                cmin = float(clr.min())
                n_inside = int((clr < foot).sum())
                print()
                print("C2 obstacle clearance over the whole flight (%d pillars from %s): minimum "
                      "pillar-surface distance %.3f m;  %d of %d samples inside the %.2f m footprint  [%s]"
                      % (len(pillars), os.path.basename(args.world), cmin, n_inside, len(clr), foot,
                         "PASS" if n_inside == 0 else "FAIL"))
        except Exception as exc:                              # a metrics tool must not die on a typo
            print("C2 not measured: %s" % exc)
    last = phases[-1]
    if last.get("cmd_x"):
        goal = np.array([float(last["cmd_x"]), float(last["cmd_y"])])
        end = np.array([float(last["truth_x"]), float(last["truth_y"])])
        d_goal = float(np.linalg.norm(end - goal))
        print("C1 final point: the aircraft stopped %.3f m from the last commanded point (%.3f, %.3f)  [%s]"
              % (d_goal, goal[0], goal[1], "PASS" if d_goal <= 0.30 else "FAIL"))
    print()
    verdict = (h_rms <= 0.020 and v_rms <= 0.050 and not aborted and ex <= 2.10 and ny <= 1.30)
    print("A1/A2/B1 %s   (A1 %.2f mm <= 20, A2 %.2f mm <= 50, B1 E %.3f <= 2.10, N %.3f <= 1.30, "
          "aborted %s)" % ("PASS" if verdict else "FAIL", h_rms * 1e3, v_rms * 1e3, ex, ny, aborted))
    return 0 if not aborted else 1


if __name__ == "__main__":
    sys.exit(main())
