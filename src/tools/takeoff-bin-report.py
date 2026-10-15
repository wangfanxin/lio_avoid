#!/usr/bin/env python3
"""What target did the FCU ADOPT?  Read from the FCU's own log, not from our telemetry.

THE ACCEPTANCE FOR STEP 1 OF F2b, MEASURED: after a take-off command the FCU's vertical position
controller aims at `CTUN.DAlt`.  `Mode::_TakeOff::start_m` (`ArduCopter/takeoff.cpp:51-57`) computes
`take_off_complete_alt_m = get_pos_estimate_U_m() + alt_m`, so the value CTUN converges on says which
datum the executor used for the number we sent:
  * ~alt_m above the pad  -> the executor read it as ALT-ABOVE-HOME (the contract it documents);
  * ~datum + alt_m        -> the executor read it as we sent it, i.e. the datum is in the target.

CTUN is a DataFlash message and exists ONLY in the FCU's BIN.  That is the whole reason this tool exists:
the live MAVLink stream cannot answer "what did the FCU decide", only "what did it do".

usage: tools/takeoff-bin-report.py <flight.BIN> --alt 2.5
"""
from __future__ import annotations

import argparse
import sys

from pymavlink import DFReader


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("binfile")
    ap.add_argument("--alt", type=float, default=2.5, help="the height above the pad that was requested")
    ap.add_argument("--tol", type=float, default=1.0, help="the acceptance bound on |adopted - requested|")
    args = ap.parse_args()

    d = DFReader.DFReader_binary(args.binfile)
    t0 = None
    ctun, orgn, modes, parms = [], [], [], {}
    while True:
        m = d.recv_msg()
        if m is None:
            break
        t = m.get_type()
        if t0 is None:
            t0 = m._timestamp
        if t == "CTUN":
            ctun.append((m._timestamp - t0, m.Alt, m.DAlt))
        elif t == "ORGN":
            orgn.append((m._timestamp - t0, m.Alt))
        elif t == "MODE":
            modes.append((m._timestamp - t0, m.Mode, m.ModeNum))
        elif t == "PARM":
            n = m.Name
            n = n.decode() if isinstance(n, bytes) else n
            parms[n] = m.Value

    print("=== the FCU's own log: %d CTUN, %d ORGN, %d MODE ===" % (len(ctun), len(orgn), len(modes)))
    print()
    print("ORGN -- the EKF origin the take-off datum is expressed against (AMSL):")
    for t, alt in orgn:
        print("   t %7.2f  origin alt %.3f m" % (t, alt))
    print()
    print("PARM -- the parameters that bound a take-off:")
    for n in ("WP_NAVALT_MIN", "PILOT_SPD_UP", "PILOT_ACC_Z", "WP_SPD_UP", "WP_ACC_Z",
              "EK3_OGN_HGT_MASK", "SIM_BARO_ALT_OFS", "ARMING_CHECK", "ARMING_SKIPCHK"):
        if n in parms:
            print("   %-18s %s" % (n, parms[n]))
    print()
    print("MODE:")
    for t, name, num in modes:
        print("   t %7.2f  %s (%d)" % (t, name, num))
    print()
    if not ctun:
        print("NO CTUN -- cannot read the adopted target from this log")
        return 1

    # The take-off window: from the first sample whose DAlt leaves the pad value, to the peak.
    pad_alt = ctun[0][1]
    moving = [r for r in ctun if r[1] > pad_alt + 0.10]
    print("CTUN  pad Alt %+.3f m; %d samples with Alt above the pad" % (pad_alt, len(moving)))
    if moving:
        print("      first movement t %7.2f (Alt %+.3f, DAlt %+.3f)"
              % (moving[0][0], moving[0][1], moving[0][2]))
    peak = max(ctun, key=lambda r: r[1])
    peak_d = max(ctun, key=lambda r: r[2])
    print("      Alt  peak %+.3f m at t %.2f   (the AIRCRAFT)" % (peak[1], peak[0]))
    print("      DAlt peak %+.3f m at t %.2f   (the TARGET the position controller aimed at)"
          % (peak_d[2], peak_d[0]))
    print()
    print("CTUN in the window (Alt / DAlt), every 5th sample -- the FCU's OWN numbers:")
    for i, (t, alt, dalt) in enumerate(ctun):
        if i % 5 == 0:
            print("   t %7.2f  Alt %+7.3f  DAlt %+7.3f" % (t, alt, dalt))
    print()

    # ============================ THE ACCEPTANCE ============================
    # "the adopted target is within ~1 m of the request, measured".  The adopted target is the value the
    # controller converges on; a runaway climbs through it and keeps going, so the test is on the
    # TARGET the FCU settled at, taken as the DAlt at the moment the Alt first reaches the request.
    hit = [r for r in ctun if r[1] >= args.alt - 0.05]
    print("=== ACCEPTANCE (adopted target vs requested height above the pad) ===")
    print("    requested            : %.3f m above the pad" % args.alt)
    if not hit:
        print("    the aircraft NEVER reached the request in this log -- max Alt %+.3f m" % peak[1])
        print("    VERDICT: NOT ACHIEVED (no target to compare)")
        return 1
    t_hit, alt_hit, dalt_hit = hit[0]
    print("    first Alt >= request : t %.2f, Alt %+.3f, DAlt %+.3f" % (t_hit, alt_hit, dalt_hit))
    err = abs(dalt_hit - args.alt)
    print("    |adopted DAlt - request| = %.3f m  (bound %.2f m)" % (err, args.tol))
    over = peak_d[2] - args.alt
    print("    the DAlt peak is %+.3f m = request %+.3f m  (overshoot %+.3f m)" % (peak_d[2], over, over))

    # THE TARGET MUST BE ADOPTED **AND HELD**.  `|adopted - request|` at the first crossing is NOT enough,
    # and that is MEASURED rather than defensive: the `datum` positive control (this tool, on
    # f2b-takeoff-datum-20261015_171522) crosses the 2.5 m request with DAlt = +2.526 -- an error of
    # 0.026 m, inside the bound -- and then CLIMBS THROUGH IT at 2.4 m/s to +32.637 m, because the target
    # the FCU adopted was the datum-scale number and the crossing was a moment on the way past.  A verdict
    # that reads only the crossing calls that "ACHIEVED" and would have hidden the whole defect.  So the
    # settled target is what is judged: the mean DAlt over the samples inside the acceptance band, plus an
    # explicit overshoot test.
    band = [r for r in ctun if abs(r[2] - args.alt) <= args.tol]
    settled = float(sum(r[2] for r in band)) / len(band) if band else float("nan")
    n_over = sum(1 for r in ctun if r[2] > args.alt + args.tol)
    print("    samples with DAlt inside the band: %d of %d; mean DAlt there %+.3f m (error %+.3f m)"
          % (len(band), len(ctun), settled, abs(settled - args.alt) if band else float("nan")))
    print("    samples with DAlt ABOVE the band : %d of %d (a runaway spends its life here)"
          % (n_over, len(ctun)))
    held = bool(band) and abs(settled - args.alt) <= args.tol
    ok = err <= args.tol and peak_d[2] <= args.alt + args.tol and held
    print("    VERDICT: %s" % ("ACHIEVED -- the adopted target IS the requested height above the pad, "
                               "held inside the bound" if ok else
                               "NOT ACHIEVED -- %s" % (
                                   "the target OVERSHOT to %+.3f m (bound %+.2f m above the request)"
                                   % (peak_d[2], args.tol) if peak_d[2] > args.alt + args.tol else
                                   ("the settled target is %.3f m from the request" % abs(settled - args.alt)
                                    if band else "no sample ever sat inside the band"))))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
