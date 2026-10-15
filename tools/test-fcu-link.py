#!/usr/bin/env python3
"""Does the FCU fly on external navigation alone?  The smallest test that can answer it.

This bypasses the LIO on purpose: the ODOMETRY stream is built from the TRUTH TAP, so if the vehicle
will not arm, take off or hold, the fault is in the FCU link (frames, source map, origin, arming) and
not in the estimator.  Only once this passes is it worth feeding the LIO's own odometry.

Sequence: connect -> set the EKF origin -> stream external nav -> GUIDED -> arm -> takeoff -> hold.
Everything printed is either a command ACK, the FCU's own state, or the truth-vs-FCU position.

usage: tools/test-fcu-link.py [--hold 15] [--alt 0.8] [--url tcp:127.0.0.1:5760] [--tap-port 14561]
"""
from __future__ import annotations

import argparse
import math
import socket
import sys
import time

import numpy as np

sys.path.insert(0, "/mnt/d/distill_self/lio_avoid")
from common import frames               # noqa: E402
from bridge.base import Mode            # noqa: E402
from bridge.mavlink_impl import MavlinkBridge   # noqa: E402
from planner import cmd_stream as _cmd_stream   # noqa: E402

HOME_LAT, HOME_LON, HOME_ALT = -35.363261, 149.165230, 584.0

#: this test's frames coincide: it feeds the FCU from the truth tap, whose frame IS F (= W, the EKF origin
#: is the pad).  Stated as a calibration rather than hidden, because since F1 the bridge's setpoint entry
#: point speaks L and nothing above it converts a frame.
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
        got = False
        while True:
            try:
                data, _ = self.sock.recvfrom(2048)
            except BlockingIOError:
                break
            f = data.decode("ascii", "replace").split()
            if len(f) != 19 or f[0] != "TAP":
                continue
            p_ned = [float(x) for x in f[3:6]]
            v_ned = [float(x) for x in f[6:9]]
            q_ned = [float(x) for x in f[9:13]]
            p_w, v_w, q_w = frames.wire_to_world(p_ned, v_ned, q_ned)
            # W (ENU, origin = pad) -> F: same labels, but the FCU's origin is where the EKF origin was
            # set, which is the pad too; the only difference is the sensor lever arm, and the truth
            # tap already reports the BODY origin, so nothing further is needed here.
            self.t_us = int(f[2])
            self.p = p_w
            self.v = v_w
            self.q = q_w
            self.n += 1
            got = True
        return got


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="tcp:127.0.0.1:5760")
    ap.add_argument("--tap-port", type=int, default=14561)
    ap.add_argument("--alt", type=float, default=0.8)
    ap.add_argument("--hold", type=float, default=15.0)
    ap.add_argument("--nav-hz", type=float, default=50.0)
    args = ap.parse_args()

    log = lambda s: print("[%7.2f] %s" % (time.monotonic() - T0, s), flush=True)
    T0 = time.monotonic()

    tap = TapSource(args.tap_port)
    br = MavlinkBridge(url=args.url, log=log)
    st = br.state()

    log("waiting for the tap (port %d)" % args.tap_port)
    t_end = time.monotonic() + 10
    while tap.n == 0 and time.monotonic() < t_end:
        tap.poll()
        time.sleep(0.05)
    if tap.n == 0:
        log("FAIL: no tap samples -- is truth_rx running with --fanout %d?" % args.tap_port)
        return 1
    log("tap alive: %d samples, truth at [%+.3f %+.3f %+.3f] F" % (tap.n, *tap.p))

    # stream external nav from here on: the EKF needs it to initialise, and the origin command only
    # succeeds once EKF3 is the ACTIVE estimator (see MavlinkBridge.set_ekf_origin)
    nav_dt = 1.0 / args.nav_hz
    next_nav = time.monotonic()

    def pump(seconds, setpoint=None):
        nonlocal next_nav
        t_end = time.monotonic() + seconds
        n = 0
        while time.monotonic() < t_end:
            tap.poll()
            br.poll()
            now = time.monotonic()
            if now >= next_nav:
                next_nav = now + nav_dt
                br.send_external_nav(tap.t_us, tap.p, tap.q, tap.v)
                n += 1
                if setpoint is not None:
                    # THE ONE SETPOINT ENTRY POINT (F1 deleted the three F-frame writers).  This bench test
                    # flies on the TRUTH tap, whose frame IS F (= W), so the frames coincide ON PURPOSE and
                    # the calibration below is the identity: the L input is the F point, verbatim.
                    br.send_cmd(_cmd_stream.CmdSample(np.asarray(setpoint, dtype=float),
                                                      np.zeros(3), np.zeros(3), 0.0, 0.0, 0,
                                                      _cmd_stream.TRAJECTORY_STATUS_READY),
                                _IDENTITY_CALIB, fields="pos")
            time.sleep(0.001)
        return n

    pump(3.0)
    ok_origin = br.set_ekf_origin(HOME_LAT, HOME_LON, HOME_ALT, pump=pump)
    log("EKF origin -> %s" % ("established" if ok_origin else "NOT established"))
    if not ok_origin:
        log("FAIL: without an origin the takeoff path cannot convert its altitude "
            "(ModeGuided::do_user_takeoff_start_m -> Location::get_alt_m(ABOVE_ORIGIN))")
        return 1
    pump(1.0)
    log("FCU state: armed=%s mode=%s pos=[%+.3f %+.3f %+.3f] ekf_ok=%s gps_fix=%d alt AMSL %.1f m"
        % (st.armed, st.mode, st.pos[0], st.pos[1], st.pos[2], st.ekf_ok, st.gps_fix, st.alt_amsl_m))

    if not br.set_mode(Mode.GUIDED):
        log("FAIL: could not enter GUIDED (statustext: %s)" % _texts(st))
        return 1
    if not br.calibrate_accel_simple():
        log("note: simple accel calibration was not ACKed (may not be needed)")
    else:
        log("simple accel calibration done")
        pump(1.0)
    if not br.arm(timeout_s=25.0):
        log("FAIL: arming refused (statustext: %s)" % _texts(st))
        pump(2.0)
        return 1
    log("armed")
    # let arming settle: the heartbeat carries the armed flag at 1 Hz, and takeoff's first guard is
    # `motors->armed()`, so commanding takeoff 50 ms after the ACK asks before the state it reads has
    # moved (MEASURED: this is exactly how the first attempts failed, with no statustext to explain it)
    pump(2.0)
    st = br.state()
    log("after arming: armed=%s mode=%s landed_state=%d (1 = on ground, 2 = in air; TAKEOFF needs 1)"
        % (st.armed, st.mode, st.landed_state))
    if not st.armed:
        log("FAIL: the FCU does not report itself armed 2 s after an ACCEPTED arm command")
        return 1

    log("armed; before takeoff: mode=%s armed=%s pos=[%+.3f %+.3f %+.3f] alt AMSL %.1f rel %.1f m"
        % (st.mode, st.armed, st.pos[0], st.pos[1], st.pos[2], st.alt_amsl_m, st.alt_rel_m))
    if st.mode != "GUIDED":
        log("mode is %s, not GUIDED -- re-entering (arming or the calibration can drop a mode)"
            % st.mode)
        br.set_mode(Mode.GUIDED)
        pump(0.5)
    if not br.takeoff(args.alt):
        log("takeoff refused (statustext: %s)" % _texts(st))
        log("re-entering GUIDED and retrying once")
        br.set_mode(Mode.GUIDED)
        pump(0.5)
        if not br.takeoff(args.alt):
            log("FAIL: takeoff refused twice (statustext: %s)" % _texts(st))
            return 1
    log("takeoff commanded to %.2f m" % args.alt)

    # WAIT FOR THE VEHICLE TO ACTUALLY LEAVE THE GROUND BEFORE STREAMING ANY SETPOINTS.
    #
    # MEASURED 2026-10-13, and it cost several runs: `MAV_CMD_NAV_TAKEOFF` is accepted and sets GUIDED's
    # SubMode::TakeOff, but every position setpoint calls `guided_set_destination()`, which REPLACES that
    # submode with WP.  Streaming setpoints at 50 Hz from the moment of the command therefore cancels
    # the takeoff: the motors never leave MOT_SPIN_ARM (1100 measured), the vehicle never leaves the
    # ground, and after DISARM_DELAY the FCU disarms itself ("Arming motors" -> "Disarming motors" in
    # its own narration).  Nothing in the telemetry says "your setpoints cancelled the takeoff".
    t_takeoff = time.monotonic()
    while time.monotonic() - t_takeoff < 12.0:
        pump(0.2)                                  # external nav only: no setpoints
        if tap.p[2] > 0.25:                        # the TRUTH, not the FCU's estimate
            log("airborne: truth altitude %.3f m after %.1f s" % (tap.p[2],
                                                                  time.monotonic() - t_takeoff))
            break
        if st.landed_state == 2:
            log("the FCU reports IN_AIR after %.1f s" % (time.monotonic() - t_takeoff))
            break
    else:
        log("FAIL: never left the ground in 12 s (motors %s, landed_state %d, armed %s)"
            % (st.servos[:4], st.landed_state, st.armed))
        return 1

    target = [0.0, 0.0, args.alt]
    t_end = time.monotonic() + args.hold
    worst = 0.0
    while time.monotonic() < t_end:
        n = pump(0.2, setpoint=(target, 0.0))
        err = float(np.linalg.norm(np.array(st.pos) - np.array(target)))
        truth_err = float(np.linalg.norm(tap.p - np.array(target)))
        worst = max(worst, err)
        if int((time.monotonic() - T0) * 2) % 4 == 0:
            # the motor outputs are printed because "takeoff accepted but nothing happens" has two very
            # different causes: no thrust commanded, or thrust commanded and not enough
            log("FCU [%+.3f %+.3f %+.3f] err %.3f | truth [%+.3f %+.3f %+.3f] | motors %s | %s | "
                "landed_state=%d"
                % (st.pos[0], st.pos[1], st.pos[2], err, tap.p[0], tap.p[1], tap.p[2],
                   " ".join("%d" % s for s in st.servos[:4]), st.mode, st.landed_state))
    log("hold done: worst FCU-vs-command error %.3f m" % worst)
    log("the FCU's own narration, last 8 messages:")
    for when, text in st.messages[-8:]:
        log("   %s" % text)

    br.land()
    pump(6.0)
    log("land commanded, mode now %s" % st.mode)
    br.close()
    return 0


def _texts(st):
    return " | ".join(t for _, t in st.messages[-5:]) or "(none)"


if __name__ == "__main__":
    sys.exit(main())
