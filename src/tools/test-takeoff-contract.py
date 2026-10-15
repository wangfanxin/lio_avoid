#!/usr/bin/env python3
"""STEP 1 OF F2b: WHICH TAKE-OFF CONTRACT DOES THE EXECUTOR ACTUALLY USE -- MEASURED, ON SITL.

THE DEFECT THIS SETTLES.  `bridge/mavlink_impl.py:259-264` sends `datum + alt_m` (586.59 m for a
2.50 m request) so that a take-off request clears the guard in `ArduCopter/takeoff.cpp:31`,
`takeoff_alt_m <= copter.current_loc.alt * 0.01`.  The SAME number is then handed to
`Mode::_TakeOff::start_m` (`takeoff.cpp:51-57`), which computes
`take_off_complete_alt_m = get_pos_estimate_U_m() + alt_m` and therefore ADDS it.  On RECT25-kill-plan
the FCU's own CTUN `DAlt` ramped at 2 m/s from +0.215 to +9.874 m on a 2.50 m command: the whole datum
was flown.  Both readings are in `_stage/f2-takeoff-datum-finding.txt`.

WHAT IS NOT SETTLED BY READING, AND IS SETTLED HERE: what NUMBER the guard wants.  The supervisor's
reading is `current_loc.alt` is an AMSL-scale quantity (~584), so the guard needs > 584 and
`MAV_CMD_NAV_TAKEOFF` "cannot take off to a small height-above-home at all".  The opposite reading is
that `current_loc.alt` is alt-above-home in CM (`ArduCopter/inertia.cpp:24-30` sets exactly that), so
`* 0.01` makes the comparison metres-above-home against metres-above-home and 2.50 passes the guard
with room to spare -- in which case the executor's own contract (alt-above-home, confirmed verbatim at
`mode_guided.cpp:157-188`) is ALSO the guard's contract, and the ONLY defect is our compensation.

ONE COMMAND DECIDES IT: send `z = 2.50` and read the FCU's verdict.  Then read what it ADOPTED, from
the FCU's OWN log (`CTUN.DAlt`, the vertical position controller's target) -- not from our telemetry.

VARIANTS (one per airborne session; each starts from disarmed, on the pad):
  clean     z = alt_m, NO setpoints                    <- the proposed fix
  datum     z = datum + alt_m, NO setpoints            <- RECT25's wire, the control that must reproduce
  setpoint  NO take-off command at all: position setpoints from the first moment (option (b))
  clean_sp  z = alt_m, setpoints once airborne          <- the sequencing the brief also allows

usage: tools/test-takeoff-contract.py --variant clean|datum|setpoint|clean_sp [--alt 2.5]
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import time

import numpy as np

sys.path.insert(0, "/mnt/d/distill_self/lio_avoid/src")
from common import frames               # noqa: E402
from bridge.base import Mode            # noqa: E402
from bridge.mavlink_impl import MavlinkBridge   # noqa: E402
from planner import cmd_stream as _cmd_stream   # noqa: E402

HOME_LAT, HOME_LON, HOME_ALT = -35.363261, 149.165230, 584.0

#: this bench feeds the FCU from the truth tap, whose frame IS F: the EKF origin is the pad, so the
#: L input is the F point verbatim.  Stated as a calibration rather than hidden (`tools/test-fcu-link.py`).
_IDENTITY_CALIB = frames.Calib(R_WL=np.eye(3), t_WL=np.zeros(3))


class TapSource:
    """The truth tap's newest sample, converted to F (ENU labels, z up, origin at the pad)."""

    def __init__(self, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", port))
        self.sock.setblocking(False)
        self.t_us = 0
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.n = 0

    def poll(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(2048)
            except BlockingIOError:
                break
            f = data.decode("ascii", "replace").split()
            if len(f) != 19 or f[0] != "TAP":
                continue
            p_w, v_w, q_w = frames.wire_to_world([float(x) for x in f[3:6]],
                                                 [float(x) for x in f[6:9]],
                                                 [float(x) for x in f[9:13]])
            self.t_us = int(f[2])
            self.p, self.v, self.q = p_w, v_w, q_w
            self.n += 1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="tcp:127.0.0.1:5760")
    ap.add_argument("--tap-port", type=int, default=14561)
    ap.add_argument("--alt", type=float, default=2.5)
    ap.add_argument("--sp-z", type=float, default=None,
                    help="WHERE THE POSITION SETPOINTS AIM, m above the pad (defaults to --alt).  The "
                         "OWNER'S take-off design is a SMALL command (0.2 m) plus setpoint-driven climb, so "
                         "the bench has to be able to ask for a 0.2 m take-off and a 2.0 m setpoint: "
                         "`--alt 0.2 --sp-z 2.0` is that flight, with the climb TRACKED rather than "
                         "adopted by the FCU's take-off controller")
    ap.add_argument("--variant", required=True,
                    choices=["clean", "datum", "setpoint", "clean_sp"])
    ap.add_argument("--seconds", type=float, default=14.0,
                    help="how long to watch after the command (or after arming, for --variant setpoint)")
    ap.add_argument("--out", default="", help="JSON record of the measurement")
    args = ap.parse_args()

    T0 = time.monotonic()

    def log(s):
        print("[%7.2f] %s" % (time.monotonic() - T0, s), flush=True)

    tap = TapSource(args.tap_port)
    br = MavlinkBridge(url=args.url, log=log)
    st = br.state()

    log("variant %s; waiting for the tap (port %d)" % (args.variant, args.tap_port))
    t_end = time.monotonic() + 10
    while tap.n == 0 and time.monotonic() < t_end:
        tap.poll()
        time.sleep(0.05)
    if tap.n == 0:
        log("FAIL: no tap samples -- is truth_rx running with --fanout %d?" % args.tap_port)
        return 1
    log("tap alive: truth at [%+.3f %+.3f %+.3f] F" % tuple(tap.p))

    nav_dt = 1.0 / 50.0
    next_nav = time.monotonic()
    sent_sp = {"n": 0}

    def pump(seconds, setpoint=None):
        nonlocal next_nav
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            tap.poll()
            br.poll()
            now = time.monotonic()
            if now >= next_nav:
                next_nav = now + nav_dt
                br.send_external_nav(tap.t_us, tap.p, tap.q, tap.v)
                if setpoint is not None:
                    br.send_cmd(_cmd_stream.CmdSample(np.asarray(setpoint, dtype=float),
                                                      np.zeros(3), np.zeros(3), 0.0, 0.0, 0,
                                                      _cmd_stream.TRAJECTORY_STATUS_READY),
                                _IDENTITY_CALIB, fields="pos")
                    sent_sp["n"] += 1
            time.sleep(0.001)

    pump(3.0)
    if not br.set_ekf_origin(HOME_LAT, HOME_LON, HOME_ALT, pump=pump):
        log("FAIL: EKF origin never accepted")
        return 1
    pump(1.0)
    if not br.set_mode(Mode.GUIDED):
        log("FAIL: could not enter GUIDED")
        return 1
    if not br.calibrate_accel_simple():
        log("note: simple accel calibration not ACKed")
    else:
        pump(1.0)
    if not br.arm(timeout_s=25.0):
        log("FAIL: arming refused: %s" % _texts(st))
        return 1
    pump(2.0)
    st = br.state()
    if not st.armed:
        log("FAIL: not armed after an ACCEPTED arm")
        return 1
    if st.mode != "GUIDED":
        log("note: mode is %s, re-entering GUIDED" % st.mode)
        br.set_mode(Mode.GUIDED)
        pump(0.5)

    # ================================ THE PRE-COMMAND MEASUREMENT ================================
    # Everything the guard's operand could be, read before the command moves anything.
    log("BEFORE the command: armed=%s mode=%s landed_state=%d alt AMSL %.3f m relative_alt %.3f m"
        % (st.armed, st.mode, st.landed_state, st.alt_amsl_m, st.alt_rel_m))
    log("                    datum (alt_amsl - alt_rel) = %.3f m; FCU local pos = [%+.3f %+.3f %+.3f] F"
        % (st.alt_amsl_m - st.alt_rel_m, st.pos[0], st.pos[1], st.pos[2]))
    log("                    the number we are about to send as packet.z: %.3f m" % args.alt)

    # ================================ THE COMMAND ================================
    ack_seen = {}
    t_cmd = time.monotonic()
    if args.variant == "setpoint":
        log("NO take-off command (option (b)): position setpoints from the first moment")
        accepted = None
    else:
        if args.variant == "datum":
            z = (st.alt_amsl_m - st.alt_rel_m) + args.alt
        else:
            z = args.alt
        log("MAV_CMD_NAV_TAKEOFF packet.z = %.3f m  (%s)"
            % (z, "datum + alt_m" if args.variant == "datum" else "the requested height, verbatim"))
        from pymavlink import mavutil
        br._last_ack = None
        br.mav.mav.command_long_send(br.mav.target_system, br.mav.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                                     0.0, 0.0, 1.0, 0.0, 0.0, 0.0, z)
        t_end = time.monotonic() + 6.0
        while time.monotonic() < t_end:
            pump(0.05)
            ack = getattr(br, "_last_ack", None)
            if ack is not None and ack[0] == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                ack_seen = {"command": int(ack[0]), "result": int(ack[1])}
                break
        if ack_seen:
            name = {0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED", 3: "UNSUPPORTED", 4: "FAILED",
                    5: "IN_PROGRESS", 6: "CANCELLED"}.get(ack_seen["result"], "?")
            accepted = ack_seen["result"] == 0
            log("COMMAND_ACK for MAV_CMD_NAV_TAKEOFF: result %d (%s)" % (ack_seen["result"], name))
            if not accepted:
                log("   the FCU's own words since we asked: %s" % _texts(st))
        else:
            accepted = None
            log("no COMMAND_ACK for MAV_CMD_NAV_TAKEOFF in 6.0 s")

    # ================================ WATCH ================================
    # The trace is the evidence: truth (z), the FCU's own altitude, the vertical target the FCU's
    # position controller is aiming at is read from its BIN afterwards -- what THIS script can see live
    # is the FCU's local position and the truth, and the climb RATE.  A runaway shows up in all three.
    trace = []
    t_air = None
    t_end = time.monotonic() + args.seconds
    sp_z = args.alt if args.sp_z is None else args.sp_z
    sp_target = (0.0, 0.0, sp_z)
    guard_hit = None
    #: the track's own summary of the CLIMB FROM THE COMMAND HEIGHT TO THE SETPOINT HEIGHT: the owner's
    #: acceptance for the setpoint-driven profile is that the climb is TRACKED (no overshoot beyond ~1 m of
    #: the target) and HELD.  Measured against the SETPOINT target, not the take-off command, when they
    #: differ -- and reported with the command's own peak separately, because "the FCU took 0.2 m" and
    #: "the setpoints flew the mission" are two different claims.
    over_sp = None
    hold_err = None
    if sp_z != args.alt:
        log("setpoint target %.3f m above the pad (the take-off command is %.3f m): the climb is the "
            "SETPOINTS' job in this variant" % (sp_z, args.alt))
    while time.monotonic() < t_end:
        want_sp = None
        if args.variant == "setpoint":
            want_sp = sp_target                       # from the first moment, no take-off command
        elif args.variant == "clean_sp":
            n_air = (tap.p[2] > 0.25) or (st.landed_state == 2) or (st.pos[2] > 0.25)
            if n_air:
                want_sp = sp_target
                if t_air is None:
                    t_air = time.monotonic() - T0
                    log("AIRBORNE (truth %.3f, FCU U %.3f, landed_state %d); setpoints ON"
                        % (tap.p[2], st.pos[2], st.landed_state))
        pump(0.1, setpoint=want_sp)
        if t_air is None and ((tap.p[2] > 0.25) or (st.landed_state == 2)):
            t_air = time.monotonic() - T0
            log("AIRBORNE: truth z %+.3f m, FCU U %+.3f m, landed_state %d, %.2f s after the command"
                % (tap.p[2], st.pos[2], st.landed_state, time.monotonic() - t_cmd))
        if tap.p[2] > args.alt + 1.0 and guard_hit is None:
            guard_hit = time.monotonic() - t_cmd
            log("*** THE CLIMB IS ABOVE THE REQUEST + 1.0 m (truth %+.3f) -- the `--climb-guard-m` "
                "condition, %.2f s after the command ***" % (tap.p[2], guard_hit))
        if sent_sp["n"] > 0 and tap.p[2] > sp_z + 1.0 and over_sp is None:
            over_sp = time.monotonic() - t_cmd
            log("*** THE SETPOINT-DRIVEN CLIMB IS ABOVE ITS TARGET + 1.0 m (truth %+.3f, target %.3f) "
                "at %.2f s -- the setpoint profile OVERSHOT ***" % (tap.p[2], sp_z, over_sp))
        trace.append({"t": time.monotonic() - T0, "truth_z": float(tap.p[2]),
                      "fcu_u": float(st.pos[2]), "amsl": float(st.alt_amsl_m),
                      "rel": float(st.alt_rel_m), "mode": st.mode, "landed": int(st.landed_state)})
    peak = max(t["truth_z"] for t in trace)
    log("watch done: truth peak %+.3f m, FCU U peak %+.3f m, setpoints sent %d"
        % (peak, max(t["fcu_u"] for t in trace), sent_sp["n"]))
    if sent_sp["n"] > 0:
        # THE HOLD: the last 3 s of the watch, against the setpoint's own target.  "Arrives and holds" is
        # the claim; the peak alone would accept a climb that never settled.
        tail = [t["truth_z"] for t in trace if t["t"] >= trace[-1]["t"] - 3.0]
        if tail:
            hold_err = float(np.mean(tail)) - sp_z
            log("setpoint profile: target %+.3f m, truth in the last 3.0 s mean %+.3f m "
                "(error %+.3f m), worst %+.3f m"
                % (sp_z, float(np.mean(tail)), hold_err, max(tail) - sp_z))
        log("SETPOINT-PROFILE VERDICT: overshoot beyond target + 1.0 m: %s; held within 0.30 m: %s"
            % ("NO" if over_sp is None else "YES at %.2f s" % over_sp,
               "YES" if hold_err is not None and abs(hold_err) <= 0.30 else "NO"))

    br.land()
    pump(6.0)
    log("land commanded; mode %s armed %s landed_state %d" % (st.mode, st.armed, st.landed_state))
    br.disarm()
    pump(2.0)
    log("final: armed=%s mode=%s" % (st.armed, st.mode))
    if args.out:
        with open(args.out, "w") as fh:
            json.dump({"variant": args.variant, "alt": args.alt, "sp_z": sp_z, "ack": ack_seen,
                       "accepted": accepted, "airborne_t": t_air, "truth_peak": peak,
                       "guard_hit_t": guard_hit, "sp_overshoot_t": over_sp, "sp_hold_err_m": hold_err,
                       "setpoints": sent_sp["n"], "trace": trace}, fh)
    br.close()
    return 0


def _texts(st):
    return " | ".join(t for _, t in st.messages[-5:]) or "(none)"


if __name__ == "__main__":
    sys.exit(main())
