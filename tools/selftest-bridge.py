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
from planner import cmd_stream                   # noqa: E402


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
    #
    # F1 NOTE: this used to call `send_position`, one of the three F-frame writers F1 deleted.  The check is
    # unchanged -- the point (1, 2, 3) in F is the point (1, 2, 3) in L under the IDENTITY calibration, so
    # the ONE remaining entry point is driven with the frames made equal ON PURPOSE, and what is asserted
    # is still the NED permutation, the frame id, the yaw and the yaw rate.
    ident = frames.Calib(R_WL=np.eye(3), t_WL=np.zeros(3))
    br = make_bridge()
    br.send_cmd(cmd_stream.CmdSample([1.0, 2.0, 3.0], np.zeros(3), np.zeros(3), math.radians(30.0),
                                     0.0, 1, 1), ident, fields="pos")
    _, args, _ = br.mav.mav.calls[-1]
    print("-- setpoints (through the ONE entry point, send_cmd, L with an identity calibration)")
    ok &= check("  setpoint position is NED (N,E,-U)", [args[5], args[6], args[7]], [2.0, 1.0, -3.0])
    ok &= check("  frame is LOCAL_NED", args[3], 1)
    ok &= check("  yaw is passed through in radians", args[14], math.radians(30.0), 1e-12)
    ok &= check("  yaw_rate is left at zero", args[15], 0.0)
    ok &= check("  fields='pos' puts POS_YAW on the wire (what the corner tour flew)",
                args[4], mi.POS_YAW_MASK)

    # ---- THE L -> WIRE CROSSING, IN THE BRIDGE (S4, and the ONE entry point since F1) ----------------
    # `send_cmd` takes a command in L and puts it on the wire, so this is where the transport's own
    # frame claim is checked: the numbers must be the same as the F-fed chain's for the same physical
    # point, and that chain is the one the old code computed (L -> W, then the NED permutation).  The
    # exhaustive, zero-tolerance A/B over three field arms and three calibrations is
    # tools/test-s4-l-to-wire.py, which CARRIES the deleted chain as its reference; what is asserted HERE
    # is the bridge's side of it, next to the wire facts it belongs with.
    calib = frames.Calib(R_WL=frames.R_z(math.radians(90.0)), t_WL=np.array([0.25, -0.5, 0.125]))
    p_l = np.array([1.0, 2.0, 3.0])
    v_l = np.array([0.4, -0.5, 0.6])
    a_l = np.array([-0.1, 0.2, -0.3])
    yaw = math.radians(30.0)
    p_w = calib.R_WL @ p_l + calib.t_WL                      # what the mission used to compute
    # the deleted chain, reproduced: `send_position_velocity(p_w, R@v_l, yaw, R@a_l)`
    br = make_bridge()
    br.mav.mav.set_position_target_local_ned_send(
        0, 1, 1, 1, mi.POS_VEL_ACC_YAW_MASK,
        float(p_w[1]), float(p_w[0]), -float(p_w[2]),
        float((calib.R_WL @ v_l)[1]), float((calib.R_WL @ v_l)[0]), -float((calib.R_WL @ v_l)[2]),
        float((calib.R_WL @ a_l)[1]), float((calib.R_WL @ a_l)[0]), -float((calib.R_WL @ a_l)[2]),
        yaw, 0.0)
    _, old_args, _ = br.mav.mav.calls[-1]
    br = make_bridge()
    sent = br.send_cmd(cmd_stream.CmdSample(p_l, v_l, a_l, yaw, 0.0, 1, 1), calib, fields="posvelacc")
    _, new_args, _ = br.mav.mav.calls[-1]
    print("-- the L -> wire crossing, in the bridge (S4)")
    ok &= check("  send_cmd returns the L point placed in F (L -> W)",
                list(sent), list(p_w), 1e-12)
    # `repr` equality, not `allclose`: the claim is BIT-identical
    same = all(repr(x) == repr(y) for x, y in zip(old_args, new_args))
    print("%-64s %s" % ("  every argument to pymavlink is the same BYTE as the old F-fed chain",
                        "ok" if same else "FAIL old=%s new=%s" % (old_args, new_args)))
    ok &= same
    ok &= check("  the L command's position reaches the wire as NED of its W image",
                [new_args[5], new_args[6], new_args[7]],
                [float(p_w[1]), float(p_w[0]), -float(p_w[2])], 1e-12)
    ok &= check("  fields='posvelacc' puts POS_VEL_ACC_YAW on the wire",
                new_args[4], mi.POS_VEL_ACC_YAW_MASK)
    br = make_bridge()
    br.send_cmd(cmd_stream.CmdSample(p_l, v_l, a_l, yaw, 0.0, 1, 1), calib, fields="posvel")
    _, pv_args, _ = br.mav.mav.calls[-1]
    ok &= check("  fields='posvel' puts POS_VEL_YAW on the wire", pv_args[4], mi.POS_VEL_YAW_MASK)
    ok &= check("  ... and its velocity crosses by the ROTATION alone (no t_WL added)",
                [pv_args[8], pv_args[9], pv_args[10]],
                [float((calib.R_WL @ v_l)[1]), float((calib.R_WL @ v_l)[0]),
                 -float((calib.R_WL @ v_l)[2])], 1e-12)
    # NEGATIVE CONTROL: the transpose the old ego adapter's self-test used.  It must move the numbers.
    bad = frames.Calib(R_WL=frames.R_z(math.radians(90.0)).T, t_WL=calib.t_WL)
    br = make_bridge()
    br.send_cmd(cmd_stream.CmdSample(p_l, v_l, a_l, yaw, 0.0, 1, 1), bad)
    _, bad_args, _ = br.mav.mav.calls[-1]
    moved = any(repr(x) != repr(y) for x, y in zip(new_args[5:8], bad_args[5:8]))
    print("%-64s %s" % ("  mutation 'R_WL transposed' moves the wire position",
                        "ok" if moved else "FAIL -- the crossing is insensitive to R_WL"))
    ok &= moved

    # ---- THE ESTIMATOR PATH'S L -> WIRE (F1) --------------------------------------------------------
    # `send_external_nav_l` is the counterpart of `send_cmd` on the estimator side: the mission used to do
    # this arithmetic itself (`p_f = R @ p_l + t`, `q_f = q_from_R(R @ R_from_q(q))`, `v_f = R @ v`) and
    # F1 composed it into the bridge so that NOTHING above the bridge converts a frame.  The claim is
    # again bit-equality with that expression.
    q_l = frames.q_from_R(frames.R_z(math.radians(40.0)))
    br = make_bridge()
    br.send_external_nav(1_000_000, calib.R_WL @ p_l + calib.t_WL,
                         frames.q_from_R(calib.R_WL @ frames.R_from_q(q_l)), calib.R_WL @ v_l)
    _, nav_old, _ = br.mav.mav.calls[-1]
    br = make_bridge()
    br.send_external_nav_l(1_000_000, p_l, q_l, v_l, calib)
    _, nav_new, _ = br.mav.mav.calls[-1]
    same = all(repr(x) == repr(y) for x, y in zip(nav_old, nav_new))
    print("-- the estimator path's L -> wire (F1)")
    print("%-64s %s" % ("  send_external_nav_l puts the same BYTES on the wire as the F-fed call",
                        "ok" if same else "FAIL old=%s new=%s" % (nav_old, nav_new)))
    ok &= same

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
