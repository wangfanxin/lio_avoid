#!/usr/bin/env python3
"""One leg, seen from inside the flight controller: is the STRAIGHTNESS failure a position-loop, an
attitude-loop or a rate-loop oscillation?

The metrics say a leg bows 50-64 mm across, with the worst point at 16-27 % of the leg and the speed
constant at 0.20-0.28 m/s.  That says "the aircraft, not the estimate" -- but the FCU's own log can say
WHICH loop, because ArduPilot logs each one separately:

  PSCN/PSCE  the position controller: target vs actual position, desired vs actual velocity
  ATT        desired vs actual roll/pitch/yaw
  RATE       desired vs actual body rates
  PIDN/PIDE  the rate controller's terms (what the loop is actually pushing with)

Everything is aligned to the truth by the take-off ramp (the same trick A4 uses), because the FCU's clock
is its own boot clock.

usage: tools/bin-leg-trace.py <flight.BIN> --tap <tap.csv> --phase <phase.csv> --leg N [--bins 24]
"""
from __future__ import annotations

import argparse
import importlib.util
import os
import sys

import numpy as np

sys.path.insert(0, "/mnt/d/distill_self/lio_avoid")
from tools.metrics import read_phase, read_tap        # noqa: E402

_spec = importlib.util.spec_from_file_location(
    "metrics_x", "/mnt/d/distill_self/lio_avoid/tools/metrics.py")
_m = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_m)


def load(path, names):
    from pymavlink import mavutil
    mlog = mavutil.mavlink_connection(path)
    out = {n: [] for n in names}
    while True:
        m = mlog.recv_match(type=list(names), blocking=False)
        if m is None:
            break
        out[m.get_type()].append(m)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bin")
    ap.add_argument("--tap", required=True)
    ap.add_argument("--phase", required=True)
    ap.add_argument("--leg", type=int, required=True)
    ap.add_argument("--bins", type=int, default=24)
    args = ap.parse_args()

    t_tap, p_tap = read_tap(args.tap)
    t_tap_ned, p_tap_ned = read_tap(args.tap, wire=True)
    phases = read_phase(args.phase)
    legs = [r for r in phases if r["phase"].startswith("leg")]
    ts = np.array([int(r.get("t_sim_us", 0)) for r in phases])
    j = phases.index(legs[args.leg - 1])
    t0 = ts[j]
    t1 = ts[j + 1] if j + 1 < len(phases) else int(t_tap[-1])

    data = load(args.bin, ["XKF1", "PSCN", "PSCE", "ATT", "RATE", "PIDN", "PIDE", "PIDR"])
    xkf = data["XKF1"]
    t_e = np.array([m.TimeUS for m in xkf], dtype=np.int64)
    pd = np.array([m.PD for m in xkf])
    # clock alignment: the same vertical-profile fit A4 uses
    m_tap = (t_tap_ned >= t_e[0] - 5_000_000) & (t_tap_ned <= t_e[-1] + 5_000_000)
    tt, td = t_tap_ned[m_tap], p_tap_ned[m_tap, 2]
    best, lag0 = None, 0
    for lag in range(-3_000_000, 3_000_001, 5000):
        prof = np.interp(t_e + lag, tt, td)
        r = float(np.sqrt(np.mean((pd - prof) ** 2)))
        if best is None or r < best:
            best, lag0 = r, lag
    t_al = t_e + lag0
    dep = -np.array([m.PD for m in xkf])          # EKF up
    print("leg %d over %.1f s; FCU clock lag %+.2f s (alignment residual %.1f mm)"
          % (args.leg, (t1 - t0) * 1e-6, lag0 * 1e-6, best * 1e3))

    def series(name, field, arr=None):
        rows = data[name]
        if arr is None:
            arr = t_al
        return arr, np.array([getattr(m, field) for m in rows])

    # truth and the EKF estimate along the leg, projected on the leg's own line
    mt = np.flatnonzero((t_tap >= t0) & (t_tap <= t1))
    seg = p_tap[mt][:, :2]
    a, b = seg[0], seg[-1]
    u = (b - a) / np.linalg.norm(b - a)
    n_hat = np.array([-u[1], u[0]])
    print("   leg direction %s; length %.3f m" % (np.round(u, 3), np.linalg.norm(b - a)))

    def cross(pos_xy, tq):
        k = np.clip(np.searchsorted(t_tap, tq), 0, len(t_tap) - 1)
        return ((pos_xy - a) @ n_hat)

    grid = np.linspace(t0, t1, args.bins + 1)[:-1]
    # the EKF's own horizontal position in the truth's frame: use its documented datum offsets
    off = np.array([np.mean(dep[np.abs(t_al - t0) < 2e6]) - np.mean(p_tap_ned[(t_tap > t0) & (t_tap < t0 + 2e6), 2])])
    ekf_up = dep - off[0]
    kk = np.clip(np.searchsorted(t_al, grid), 0, len(t_al) - 1)
    cross_ekf = (np.column_stack([np.array([m.PE for m in xkf])[kk], np.array([m.PN for m in xkf])[kk]]) - a) @ n_hat
    cross_tru = (np.column_stack([np.interp(grid, t_tap, p_tap[:, 0]), np.interp(grid, t_tap, p_tap[:, 1])]) - a) @ n_hat
    print("   along-leg: " + " ".join("%6.1f" % v for v in (grid - t0) * 1e-6))
    print("   truth[mm] " + " ".join("%6.1f" % (v * 1e3) for v in cross_tru))
    print("   EKF  [mm] " + " ".join("%6.1f" % (v * 1e3) for v in cross_ekf))
    for nm, msg, flds, scale in (("ATT roll", "ATT", ("Roll", "DesRoll"), 1.0),
                                 ("ATT pitch", "ATT", ("Pitch", "DesPitch"), 1.0),
                                 ("ATT yaw", "ATT", ("Yaw", "DesYaw"), 1.0),
                                 ("RATE r", "RATE", ("R", "RDes"), 1.0),
                                 ("RATE y", "RATE", ("Y", "YDes"), 1.0),
                                 ("PSCE VE", "PSCE", ("VE", "TVE"), 100.0),
                                 ("PSCN VN", "PSCN", ("VN", "TVN"), 100.0)):
        rows = data[msg]
        tt2 = np.array([m.TimeUS + lag0 for m in rows])
        vals = []
        for f in flds:
            v = np.array([getattr(m, f) for m in rows]) * scale
            vals.append(np.interp(grid, tt2, v))
        print("   %-9s " % nm + " ".join("%6.2f" % v for v in vals[0]) + "   (desired)")
        print("   %-9s " % "" + " ".join("%6.2f" % v for v in vals[1]))
    # the frequency of the cross-track wobble
    if len(mt) > 200:
        tq = t_tap[mt] * 1e-6
        sig = cross_tru
        sig = sig - sig.mean()
        fps = 1.0 / np.mean(np.diff(t_tap[mt] * 1e-6))
        f = np.fft.rfftfreq(len(sig), 1.0 / fps)
        P = np.abs(np.fft.rfft(sig * np.hanning(len(sig)))) ** 2
        keep = (f > 0.05) & (f < 3.0)
        if keep.any():
            print("   dominant cross-track frequency: %.2f Hz (period %.2f s)"
                  % (f[keep][np.argmax(P[keep])], 1.0 / f[keep][np.argmax(P[keep])]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
