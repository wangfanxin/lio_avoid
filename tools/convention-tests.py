#!/usr/bin/env python3
"""The convention tests of FRAMES.md 5 that need no flight.

Each test pins something the others are blind to, and each is run twice: once as it is, and once with
a deliberately wrong conversion that must make it FAIL.  That second run is the whole point -- the S4
gate taught this the hard way (a check that passed with a level housing, i.e. with no power at all).

  T2  the tap's own attitude at the spawn heading, converted to W with `common/frames.py`, must give
      the heading the simulator was told to spawn at (compass: 0 = North, +90 = East).  A sign slip in
      the NED<->ENU permutation or in the FRD->FLU roll shows up here and nowhere else.
  T3  parked and level: the gravity vector as the IMU sees it (in the housing frame S) must be
      45.000 deg away from the housing's +z axis, and the BODY pitch from the tap must be 0 deg.  That
      45 deg is the mount, measured from the sensor side; 0 deg is the body, and the distance between
      the two numbers is what stops a sensor attitude from being fed to the FCU as the body's (T3b,
      which needs the LIO in the loop and therefore belongs to S6/S7).

usage:
  tools/convention-tests.py --tap-port 14561 --spawn-heading 90 --mav tcp:127.0.0.1:5760
  tools/convention-tests.py --selftest          # offline: tests the tests with synthetic data
"""
from __future__ import annotations

import argparse
import math
import socket
import sys
import time

import numpy as np

sys.path.insert(0, "/mnt/d/distill_self/lio_avoid")
from common import frames          # noqa: E402

TOL_HEADING_DEG = 0.5
TOL_GRAVITY_DEG = 0.5
TOL_PITCH_DEG = 0.5
EXPECTED_SpecificForce_ANGLE_DEG = 45.0     # the mount, seen by the IMU (the old stack: 45.001)


class Tap:
    """Collects the raw tap lines for a while, keeping the fields the tests need."""

    def __init__(self, port, seconds):
        self.p_ned = []
        self.v_ned = []
        self.q_ned = []
        self.gyro = []
        self.accel = []
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("127.0.0.1", port))
        sock.settimeout(0.3)
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            try:
                data, _ = sock.recvfrom(2048)
            except socket.timeout:
                continue
            f = data.decode("ascii", "replace").split()
            if len(f) != 19 or f[0] != "TAP":
                continue
            self.p_ned.append([float(x) for x in f[3:6]])
            self.v_ned.append([float(x) for x in f[6:9]])
            self.q_ned.append([float(x) for x in f[9:13]])
            self.gyro.append([float(x) for x in f[13:16]])
            self.accel.append([float(x) for x in f[16:19]])
        sock.close()
        for name in ("p_ned", "v_ned", "q_ned", "gyro", "accel"):
            setattr(self, name, np.array(getattr(self, name), dtype=float))

    def ok(self):
        return len(self.q_ned) > 10


def heading_of_tap(tap) -> float:
    """The vehicle's heading in W, in compass degrees, from the tap's body-FRD->NED quaternion."""
    q = tap.q_ned.mean(axis=0)
    q = q / np.linalg.norm(q)
    _, _, q_w = frames.wire_to_world([0, 0, 0], [0, 0, 0], q)
    R_wb = frames.R_from_q(q_w)
    yaw_enu = math.degrees(math.atan2(R_wb[1, 0], R_wb[0, 0]))     # 0 = East, +CCW
    return (90.0 - yaw_enu) % 360.0                                # compass: 0 = North, +CW


def body_pitch_deg(tap) -> float:
    q = tap.q_ned.mean(axis=0)
    q = q / np.linalg.norm(q)
    R = frames.R_from_q(q)                     # body FRD -> NED
    return math.degrees(math.asin(max(-1.0, min(1.0, -R[2, 0]))))


def roll_pitch_in_world(tap):
    """Roll and pitch of the body in W, through the full conversion (wire -> W).

    This is what catches the 180 deg FRD/FLU roll and any double conversion: the heading is blind to
    them (the roll is about the body's x axis, so the nose still points the same way), but the vehicle
    is then reported rolled over.  MEASURED 2026-10-13 by the self-test: dropping F_FRD_FLU moves the
    heading by exactly 0.00 deg and this by 180.00 deg, which is why the heading test alone is not
    enough.
    """
    q = tap.q_ned.mean(axis=0)
    q = q / np.linalg.norm(q)
    _, _, q_w = frames.wire_to_world([0, 0, 0], [0, 0, 0], q)
    R = frames.R_from_q(q_w)
    roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -R[2, 0]))))
    return roll, pitch


def specific_force_angle_in_sensor(tap, R_body_sensor) -> float:
    """Angle between the measured SPECIFIC FORCE (in S) and the housing's +z axis.

    At rest, level, an accelerometer measures +g along the body's UP axis, so with the housing pitched
    45 deg nose-down the housing's +z reads cos(45) of g and the angle is 45.000 deg.  That is the
    number the flight stack measured parked (45.001 deg), and it is the mount seen from the sensor
    side.  (The angle to the GRAVITY direction, i.e. the negated vector, would be 135 deg -- the first
    version of this test asserted 45 for that quantity and the self-test caught it.)
    """
    a_frd = tap.accel.mean(axis=0)
    a_flu = frames.frd_to_flu(a_frd)
    a_s = R_body_sensor.T @ a_flu
    z_s = np.array([0.0, 0.0, 1.0])
    return math.degrees(math.acos(max(-1.0, min(1.0, float(a_s @ z_s / np.linalg.norm(a_s))))))


def fcu_attitude(mav_url, seconds=3.0, ask_hz=10.0):
    """The FCU's own reported attitude, for the record.  Parked with no aiding its YAW is not
    observable, so yaw is reported but not gated here (that is T2b, and it needs the LIO).

    MEASURED 2026-10-13: SITL's `SR*_*` stream-rate parameters are ~0 out of the box, so a client sees
    only HEARTBEAT and SYS_STATUS until it ASKS.  The ask is `MAV_CMD_SET_MESSAGE_INTERVAL` (511),
    which is also how the bridge will request the messages that are in no stream group at all (msg 31).
    """
    try:
        from pymavlink import mavutil
    except ImportError:
        return None
    try:
        m = mavutil.mavlink_connection(mav_url)
        m.wait_heartbeat(timeout=5)
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                                int(1e6 / ask_hz), 0, 0, 0, 0, 0)
    except Exception as e:                                    # pragma: no cover
        print("  (no FCU telemetry: %s)" % e)
        return None
    att = []
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        msg = m.recv_match(type="ATTITUDE", blocking=True, timeout=0.5)
        if msg is not None:
            att.append([msg.roll, msg.pitch, msg.yaw, msg.get_srcSystem()])
    m.close()
    if not att:
        return None
    a = np.array(att, dtype=float)
    return {"roll_deg": math.degrees(float(np.mean(a[:, 0]))),
            "pitch_deg": math.degrees(float(np.mean(a[:, 1]))),
            "yaw_deg": math.degrees(float(np.mean(a[:, 2]))) % 360.0,
            "samples": len(a),
            "rate_hz": len(a) / max(1e-9, seconds)}


def _mutate(kind):
    """Deliberately wrong conversions, applied in memory, for the power runs."""
    if kind == "none":
        return
    if kind == "flip-enu":
        # the classic: treat NED->ENU as no swap at all
        frames.P_NED_ENU = np.eye(3)
    elif kind == "no-roll":
        # forget the 180 deg FRD -> FLU roll
        frames.F_FRD_FLU = np.eye(3)
    elif kind == "level-mount":
        frames.R_body_sensor = lambda *a, **k: np.eye(3)
    elif kind == "noseup-mount":
        frames.R_body_sensor = lambda mount_pitch_deg=45.0, **k: frames.R_y(-math.radians(mount_pitch_deg))
    else:
        raise SystemExit("unknown mutation %r" % kind)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--tap-port", type=int, default=14561)
    ap.add_argument("--spawn-heading", type=float, required=False, default=None,
                    help="the compass heading the simulator was told to spawn at")
    ap.add_argument("--mav", default="tcp:127.0.0.1:5760")
    ap.add_argument("--seconds", type=float, default=4.0)
    ap.add_argument("--mutation", default="none",
                    choices=["none", "flip-enu", "no-roll", "level-mount", "noseup-mount"])
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        return _selftest()

    _mutate(args.mutation)
    tap = Tap(args.tap_port, args.seconds)
    print("tap samples    : %d" % len(tap.q_ned))
    if not tap.ok():
        print("FAIL: no tap data on port %d -- is truth_rx running with --fanout?" % args.tap_port)
        return 1

    R_BS = frames.R_body_sensor()
    heading = heading_of_tap(tap)
    pitch = body_pitch_deg(tap)
    roll_w, pitch_w = roll_pitch_in_world(tap)
    g_angle = specific_force_angle_in_sensor(tap, R_BS)
    fcu = fcu_attitude(args.mav)

    print("mutation       : %s" % args.mutation)
    print("heading (tap)  : %.3f deg compass" % heading)
    print("body pitch     : %+.3f deg (from the NED quaternion alone)" % pitch)
    print("in W           : roll %+.3f deg, pitch %+.3f deg (through frames.wire_to_world)"
          % (roll_w, pitch_w))
    print("specific force : %.3f deg from the housing's +z (expected %.3f -- the mount)"
          % (g_angle, EXPECTED_SpecificForce_ANGLE_DEG))
    if fcu:
        print("FCU ATTITUDE   : roll %+.2f pitch %+.2f yaw %.2f deg (NED) over %d samples at %.1f Hz "
              "-- yaw is reported, not gated, because with no aiding the EKF cannot observe it (T2b "
              "needs the LIO feeding vision)"
              % (fcu["roll_deg"], fcu["pitch_deg"], fcu["yaw_deg"], fcu["samples"], fcu["rate_hz"]))
    else:
        print("FCU ATTITUDE   : unavailable (asked for msg 30 at 10 Hz via command 511)")

    results = []
    if args.spawn_heading is not None:
        err = abs(((heading - args.spawn_heading + 180.0) % 360.0) - 180.0)
        results.append(("T2a tap heading == spawn heading", err <= TOL_HEADING_DEG,
                        "|%.3f - %.3f| = %.3f deg (<= %.1f)" % (heading, args.spawn_heading, err,
                                                                TOL_HEADING_DEG)))
    results.append(("T2b body is level in W at the pad",
                    abs(roll_w) <= TOL_PITCH_DEG and abs(pitch_w) <= TOL_PITCH_DEG,
                    "roll %+.3f, pitch %+.3f deg (both <= %.1f; catches the 180 deg FRD/FLU roll and "
                    "any double conversion)" % (roll_w, pitch_w, TOL_PITCH_DEG)))
    err_g = abs(g_angle - EXPECTED_SpecificForce_ANGLE_DEG)
    results.append(("T3a specific force in S is the mount angle", err_g <= TOL_GRAVITY_DEG,
                    "|%.3f - %.1f| = %.3f deg (<= %.1f)" % (g_angle,
                                                            EXPECTED_SpecificForce_ANGLE_DEG,
                                                            err_g, TOL_GRAVITY_DEG)))
    results.append(("T3b body pitch is 0 while the sensor reads 45", abs(pitch) <= TOL_PITCH_DEG,
                    "|%.3f| = %.3f deg (<= %.1f)" % (pitch, abs(pitch), TOL_PITCH_DEG)))

    print()
    ok_all = True
    for name, ok, detail in results:
        ok_all = ok_all and ok
        print("  [%s] %-44s %s" % ("PASS" if ok else "FAIL", name, detail))
    print("VERDICT: %s" % ("PASS" if ok_all else "FAIL"))
    return 0 if ok_all else 1


def _selftest() -> int:
    """Offline: the tests must pass on synthetic truth and FAIL on each mutation.

    The synthetic tap is built from EXPLICIT matrices, not by calling frames.world_to_wire: the first
    version of this self-test generated its data with the same mutable module globals the tests read,
    so a mutated conversion cancelled itself out and the mutations showed "no power" while the code was
    fine.  Test data must not depend on the thing under test.
    """
    ok = True

    def synth(heading_deg, mount_pitch, body_pitch_deg=0.0, n=200):
        t = Tap.__new__(Tap)
        # In NED/FRD: a compass heading h is a rotation about NED +z (North towards East) by +h, and a
        # nose-up pitch p is a rotation about the FRD y axis by +p (body x becomes forward AND up,
        # because NED z points down).
        R_ned_frd = frames.R_z(math.radians(heading_deg)) @ frames.R_y(math.radians(body_pitch_deg))
        q_ned = frames.q_from_R(R_ned_frd)
        t.q_ned = np.tile(q_ned, (n, 1))
        # at rest the accelerometer reads -g along the FRD z axis (measured on the tap in S3: -9.8067)
        t.accel = np.tile(np.array([0.0, 0.0, -9.80665]), (n, 1))
        t.p_ned = np.zeros((n, 3))
        t.v_ned = np.zeros((n, 3))
        t.gyro = np.zeros((n, 3))
        return t

    def check(name, got, want, tol):
        nonlocal ok
        good = abs(got - want) <= tol
        ok = ok and good
        print("%-58s %s" % (name, "ok" if good else "FAIL got=%.3f want=%.3f" % (got, want)))

    tap = synth(0.0, 45.0)
    check("synthetic heading 0 -> North", heading_of_tap(tap), 0.0, 1e-6)
    check("synthetic heading 90", heading_of_tap(synth(90.0, 45.0)), 90.0, 1e-6)
    check("synthetic heading 270", heading_of_tap(synth(270.0, 45.0)), 270.0, 1e-6)
    check("specific force in S with a 45 deg mount",
          specific_force_angle_in_sensor(tap, frames.R_body_sensor()), 45.0, 1e-6)
    check("specific force with a LEVEL housing",
          specific_force_angle_in_sensor(tap, frames.R_body_sensor(mount_pitch_deg=0.0)), 0.0, 1e-6)
    check("body pitch from the tap (nose up 10)",
          body_pitch_deg(synth(0.0, 45.0, body_pitch_deg=10.0)), 10.0, 1e-6)
    check("body pitch from the tap (nose down 5)",
          body_pitch_deg(synth(0.0, 45.0, body_pitch_deg=-5.0)), -5.0, 1e-6)
    check("body level in W (roll)", roll_pitch_in_world(tap)[0], 0.0, 1e-6)
    check("body level in W (pitch)", roll_pitch_in_world(tap)[1], 0.0, 1e-6)

    # the mutations must move the measured numbers, or the tests have no power
    saved_p = frames.P_NED_ENU
    _mutate("flip-enu")
    moved_enu = abs(heading_of_tap(synth(90.0, 45.0)) - 90.0)
    frames.P_NED_ENU = saved_p

    saved_f = frames.F_FRD_FLU
    _mutate("no-roll")
    moved_roll = abs(roll_pitch_in_world(synth(90.0, 45.0))[0])
    frames.F_FRD_FLU = saved_f

    saved_rbs = frames.R_body_sensor
    _mutate("level-mount")
    moved_mount = abs(specific_force_angle_in_sensor(synth(0.0, 45.0), frames.R_body_sensor()) - 45.0)
    frames.R_body_sensor = saved_rbs

    for name, moved in (("flip-enu moves the heading", moved_enu),
                        ("no-roll rolls the body in W", moved_roll),
                        ("level-mount moves the mount angle", moved_mount)):
        good = moved > 1.0
        ok = ok and good
        print("%-58s %s" % ("mutation %s by %.2f deg" % (name, moved),
                            "ok" if good else "FAIL (no power)"))

    print("convention selftest:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
