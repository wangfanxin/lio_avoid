#!/usr/bin/env python3
"""What does the bridge actually put on the wire?  Tested with a recorder, no simulator.

Written after a real flight loss: `send_external_nav` converted the position to NED but sent the
quaternion unconverted, so the FCU believed a North-facing vehicle faced East, the position loop drove
it sideways, the EKF failsafe fired and it hit the ground.  Every other check in the project passed,
because `common/frames.py` was right -- the mistake was that the bridge did not USE it for one field.

So this test reads the arguments the bridge passes to pymavlink and asserts the numbers, in the FRD/FLU
and NED/ENU senses, for two attitudes (0 and +90 deg of yaw) -- the second is what catches an
unconverted quaternion, since at yaw 0 the two conventions coincide in the yaw channel and the error is
invisible.  Each case has a deliberately wrong variant that must fail.

usage: tools/selftest-bridge.py
"""
from __future__ import annotations

import math
import sys

import numpy as np

sys.path.insert(0, "/mnt/d/distill_self/lio_avoid")
from common import frames                        # noqa: E402
from bridge import mavlink_impl as mi            # noqa: E402


class Recorder:
    """Stands in for pymavlink's MAVLink object: records the calls, returns nothing."""

    def __init__(self):
        self.calls = []

    def __getattr__(self, name):
        def rec(*args, **kwargs):
            self.calls.append((name, args, kwargs))
        return rec


class FakeConn:
    def __init__(self):
        self.mav = Recorder()
        self.target_system = 1
        self.target_component = 1


def make_bridge(mutate=None):
    """A MavlinkBridge with no socket, plus an optional in-memory mutation of frames.py."""
    if mutate == "no-attitude-conversion":
        # the actual bug: pretend the ENU/FLU quaternion may be sent as-is
        frames.world_to_wire = lambda p, v, q: (np.asarray([p[1], p[0], -p[2]]),
                                                np.asarray([v[1], v[0], -v[2]]), np.asarray(q))
    elif mutate == "position-unswapped":
        frames.world_to_wire = lambda p, v, q: (np.asarray([p[0], p[1], -p[2]]),
                                                np.asarray([v[1], v[0], -v[2]]),
                                                _wire_q(p, v, q))
    br = mi.MavlinkBridge.__new__(mi.MavlinkBridge)      # bypass __init__ (no socket)
    br.mav = FakeConn()
    br.log = lambda s: None
    br._sent_positions = 0
    br._sent_nav = 0
    return br


def _wire_q(p, v, q):
    return frames.world_to_wire.__wrapped__(p, v, q)[2] if hasattr(frames.world_to_wire, "__wrapped__") \
        else frames.q_from_R(frames.P_NED_ENU @ frames.R_from_q(q) @ frames.F_FRD_FLU)


def check(name, got, want, tol=1e-9):
    good = np.allclose(np.asarray(got, dtype=float), np.asarray(want, dtype=float), atol=tol)
    print("%-64s %s" % (name, "ok" if good else "FAIL got=%s want=%s" % (got, want)))
    return good


def main() -> int:
    ok = True
    # ENU yaw psi (measured from East, anticlockwise) -> compass/NED heading = 90 - psi.
    # Stated here because getting it the other way round is exactly how this test first "failed" while
    # the code was right; see FRAMES.md 0.3 on the compass-vs-ENU trap.
    for psi_deg in (0.0, 90.0, 180.0, 270.0):
        psi = math.radians(psi_deg)
        q_w = frames.q_from_R(frames.R_z(psi))                 # body FLU -> ENU
        want_heading = (90.0 - psi_deg) % 360.0
        br = make_bridge()
        br.send_external_nav(1_000_000, [1.0, 2.0, 3.0], q_w, [4.0, 5.0, 6.0])
        name, args, _ = br.mav.mav.calls[-1]
        _, frame, child, x, y, z, q, vx, vy, vz = args[:10]
        print("-- ENU yaw %+.0f deg (compass heading %.0f deg)" % (psi_deg, want_heading))
        ok &= check("  odometry position is NED (N,E,-U) of (E,N,U)=(1,2,3)", [x, y, z], [2.0, 1.0, -3.0])
        ok &= check("  frame pair is LOCAL_FRD / BODY_FRD", [frame, child], [20, 12])
        R_ned_frd = frames.R_from_q(q)
        got_heading = math.degrees(math.atan2(R_ned_frd[1, 0], R_ned_frd[0, 0])) % 360.0
        ok &= check("  attitude heading in NED = %.0f deg" % want_heading, got_heading, want_heading,
                    1e-6)
        # velocity: world (E,N,U) = (4,5,6), expressed in BODY FRD
        v_world_ned = np.array([5.0, 4.0, -6.0])
        want_v = R_ned_frd.T @ v_world_ned
        ok &= check("  velocity is the world velocity in BODY FRD", [vx, vy, vz], want_v, 1e-9)

    # position setpoints: F (E,N,U) -> NED, and the yaw is passed through in NED convention.
    # argument order of set_position_target_local_ned_send:
    #   0 time, 1 sys, 2 comp, 3 frame, 4 mask, 5 x, 6 y, 7 z, 8..10 v, 11..13 af, 14 yaw, 15 yaw_rate
    br = make_bridge()
    br.send_position([1.0, 2.0, 3.0], math.radians(30.0))
    _, args, _ = br.mav.mav.calls[-1]
    print("-- setpoints")
    ok &= check("  setpoint position is NED (N,E,-U)", [args[5], args[6], args[7]], [2.0, 1.0, -3.0])
    ok &= check("  frame is LOCAL_NED", args[3], 1)
    ok &= check("  yaw is passed through in radians", args[14], math.radians(30.0), 1e-12)
    ok &= check("  yaw_rate is left at zero", args[15], 0.0)

    # the mutations must move the numbers that matter
    saved = frames.world_to_wire
    br = make_bridge(mutate="no-attitude-conversion")
    q_w = frames.q_from_R(frames.R_z(math.radians(90.0)))      # compass heading 0, i.e. North
    br.send_external_nav(1_000_000, [1.0, 2.0, 3.0], q_w)
    _, args, _ = br.mav.mav.calls[-1]
    R = frames.R_from_q(args[6])
    heading = math.degrees(math.atan2(R[1, 0], R[0, 0])) % 360.0
    # unconverted, the FCU reads compass heading 90 (East) for a vehicle facing North (0)
    ok &= check("mutation 'no attitude conversion' makes North read as East", heading, 90.0, 1e-6)
    frames.world_to_wire = saved

    br = make_bridge(mutate="position-unswapped")
    br.send_external_nav(1_000_000, [1.0, 2.0, 3.0], frames.q_from_R(np.eye(3)))
    _, args, _ = br.mav.mav.calls[-1]
    ok &= check("mutation 'position unswapped' leaves x,y in ENU order", [args[3], args[4]],
                [1.0, 2.0], 1e-9)
    frames.world_to_wire = saved

    print("bridge selftest:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
