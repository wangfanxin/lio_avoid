#!/usr/bin/env python3
"""Every frame conversion in the project, in one place.

Frames (see docs/FRAMES.md; six exist, three are reasoned about at once):

  W   world, ENU, z up, origin = the pad.  This is the frame `worlds/pillars.yaml` is written in
      and the frame RViz's Fixed Frame points at.
  L   the LIO's world: z up, yaw fixed by the LIO's own initialisation, origin = the LIO's first
      pose.  Super-LIO starts at identity, so L's axes are the IMU/housing axes at init.
  B   body, FLU (ROS `base_link`).
  F   the FCU's local frame as *our transport* presents it (PLAN.md decision 4): ENU labels, z up,
      origin = the EKF origin.  On the wire it is NED -- the bridge is the one place that changes.
  S   the sensor housing (MID-360).  The cloud and the IMU are **both** published in S, exactly as
      `livox_ros_driver2` does on the real aircraft, which is why the LIO's `lidar_imu` is identity
      and the 45 deg mount lives in `odom_robo` (src/lio-fork-local/config/livox_360.yaml).
  wire  the truth tap: SITL-native NED position/velocity, body FRD quaternion, body gyro and
      specific force in FRD (PLAN.md decision 13).

Conventions: vectors are numpy (3,) float64, rotations are 3x3 matrices, quaternions are
scalar-first (w, x, y, z) Hamilton, all right-handed.  Nothing here silently re-orders an axis:
each function says which two frames it maps between.

Every conversion in this file is exercised by tools/conv-test-*.py, and each test is also run with
a deliberately flipped sign so that a test which cannot fail is visible as such (PLAN.md rule 5).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# NED -> ENU: (E, N, U) = (N, E, -U).  Its own inverse, and it is a rotation (det = +1).
P_NED_ENU = np.array([[0.0, 1.0, 0.0],
                      [1.0, 0.0, 0.0],
                      [0.0, 0.0, -1.0]])

# body FRD (SITL, MAVLink) -> body FLU (ROS): F = diag(1, -1, -1), a 180 deg roll about +x.
F_FRD_FLU = np.diag([1.0, -1.0, -1.0])


# --------------------------------------------------------------------------- vectors
def ned_to_enu(v):
    """(N, E, D) -> (E, N, U)."""
    return P_NED_ENU @ np.asarray(v, dtype=float)


def enu_to_ned(v):
    """(E, N, U) -> (N, E, D).  The same permutation."""
    return P_NED_ENU @ np.asarray(v, dtype=float)


def frd_to_flu(v):
    """body FRD -> body FLU: x forward stays, y right -> left, z down -> up."""
    return F_FRD_FLU @ np.asarray(v, dtype=float)


def flu_to_frd(v):
    """body FLU -> body FRD.  The same matrix."""
    return F_FRD_FLU @ np.asarray(v, dtype=float)


# --------------------------------------------------------------------------- rotations
def q_from_R(R):
    """Rotation matrix -> quaternion (w, x, y, z), Hamilton, scalar first."""
    R = np.asarray(R, dtype=float)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z], dtype=float)
    return q / np.linalg.norm(q)


def R_from_q(q):
    """Quaternion (w, x, y, z) -> rotation matrix."""
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q)
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_y(angle_rad):
    """Rotation about +y (the FLU pitch axis) by `angle_rad`."""
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def R_z(angle_rad):
    """Rotation about +z (the FLU/ENU yaw axis) by `angle_rad`."""
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def R_WB_from_heading(heading_deg):
    """Body -> W for a vehicle whose nose points at `heading_deg` **in compass terms**.

    Compass: 0 = North, +90 = East (so it increases clockwise when seen from above).  W is ENU, so
    its own yaw increases anticlockwise from East: W yaw = 90 - heading.  Both readings appear in
    this project's documents, which is exactly why this function exists and why `R_z` (ENU yaw) and
    `R_WB_from_heading` (compass heading) are never the same argument.
    """
    return R_z(math.radians(90.0 - heading_deg))


def yaw_from_R(R):
    """Yaw (heading about +z) of a rotation matrix, radians, wrapped to (-pi, pi]."""
    R = np.asarray(R, dtype=float)
    return math.atan2(R[1, 0], R[0, 0])


def wrap_pi(a):
    """Wrap an angle to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# --------------------------------------------------------------------------- the wire
def wire_to_world(p_ned, v_ned, q_ned_wxyz):
    """Truth tap (NED position/velocity, body-FRD -> NED quaternion) to W (ENU, body FLU).

    Returns (p_w, v_w, q_w) where q_w rotates body FLU into W.  This is the **one** place the wire
    frame is crossed: everything downstream of this function is W / body-FLU.
    """
    p_w = ned_to_enu(p_ned)
    v_w = ned_to_enu(v_ned)
    R_w_b = P_NED_ENU @ R_from_q(q_ned_wxyz) @ F_FRD_FLU
    return p_w, v_w, q_from_R(R_w_b)


def world_to_wire(p_w, v_w, q_w_wxyz):
    """Inverse of `wire_to_world`: W / body-FLU back to the tap's NED / FRD."""
    p_ned = enu_to_ned(p_w)
    v_ned = enu_to_ned(v_w)
    R_ned_frd = P_NED_ENU @ R_from_q(q_w_wxyz) @ F_FRD_FLU
    return p_ned, v_ned, q_from_R(R_ned_frd)


# --------------------------------------------------------------------------- L -> wire
def l_to_wire(l, calib):
    """L -> the WIRE frame, in ONE place: `permute(R_WL @ p_L + t_WL)`.

    WHY THIS FUNCTION EXISTS.  `docs/ARCHITECTURE.md:206` and `bridge/base.py` both say the transport
    "crosses the frame boundary exactly once", and until 2026-10-16 the mission's command did not: the
    chain computed the position in THREE places -- `fsm/mission_fsm.py:446-449` did the L -> W step
    (calling the result `p_f`) and `bridge/mavlink_impl.py:345`, `:361`, `:369` each did the W -> NED
    permutation.  `F` in the code is `W` numerically (the EKF origin is the pad), so the two steps
    happened to compose correctly, but "happened to" is not a contract: a second implementation of
    either step, or a different EKF origin, and the double conversion that `bridge/base.py` says this
    file exists to prevent would be exactly what happened.

    `l` may be a position, a velocity or an acceleration -- they are all vectors in L and all cross with
    the same rotation and offset.  A velocity crosses by the rotation alone; the caller does that, and
    `bridge.mavlink_impl.send_cmd` documents where.

    The rounding note that makes the S4 A/B provable with `atol = 0.0`: `Calib` is applied as
    `R_WL @ p_L + t_WL` and the axis permutation is applied AFTERWARDS, by the caller.  Composing the
    two into one `P_NED_ENU @ np.column_stack([R_WL, t_WL])` matrix would be mathematically identical
    and NOT bit-identical, because the sum would accumulate in a different order; keeping them separate
    is what lets the relocated expression be compared to the old one exactly (S4 of
    `_stage/refactor-bridge-plan.md` section 6).
    """
    R, t = calib.R_WL, calib.t_WL
    return np.asarray(R, dtype=float) @ np.asarray(l, dtype=float) + np.asarray(t, dtype=float)


def l_to_wire_velocity(v_l, calib):
    """L -> wire for a VELOCITY: the same rotation, and NO translation.

    A velocity is a difference of two positions, so `Calib.t_WL` cancels: applying it here would add
    about 0.1 m/s to a vehicle that is standing still, which is exactly the kind of "silently the wrong
    number" this file is written to make impossible.  Kept as its own function rather than a `kind=`
    switch so that a caller cannot get it wrong by passing the wrong flag.
    """
    return np.asarray(calib.R_WL, dtype=float) @ np.asarray(v_l, dtype=float)


# --------------------------------------------------------------------------- the sensor
def R_body_sensor(mount_pitch_deg=45.0, mount_roll_deg=0.0, mount_yaw_deg=0.0):
    """B (FLU) <- S: the housing's pose in the body frame.

    A nose-DOWN mount of `mount_pitch_deg` means the housing's forward axis lies
    `mount_pitch_deg` below the body's forward axis: in body coordinates the housing's x axis is
    (cos, 0, -sin), which is the first column of R_y(+pitch).  So the physical statement "45 deg
    nose-down" is R_y(+45), and the housing's up axis then tilts forward -- as a head does when its
    owner bends forward.

    The flight stack states the same mount as `odom_robo` pitch = +45 with its rpy applied
    INVERTED (src/lio-fork-local/config/livox_360.yaml), i.e. as Ry(-45) mapping body -> sensor.
    tools/conv-test-t5.py measures this sign against the world rather than trusting either reading.
    """
    R_roll = np.array([[1.0, 0.0, 0.0],
                       [0.0, math.cos(math.radians(mount_roll_deg)),
                        -math.sin(math.radians(mount_roll_deg))],
                       [0.0, math.sin(math.radians(mount_roll_deg)),
                        math.cos(math.radians(mount_roll_deg))]])
    return R_z(math.radians(mount_yaw_deg)) @ R_y(math.radians(mount_pitch_deg)) @ R_roll


SENSOR_LEVER_ARM_B = np.array([0.05, 0.0, 0.10])  # body -> housing, metres (measured hardware fact)


# --------------------------------------------------------------------------- calibration
@dataclass
class Calib:
    """The measured L -> W transform, checked against what the construction predicts.

    `R_WL` maps L coordinates to W coordinates (v_W = R_WL @ v_L), so its first column is L's x axis
    written in W, and its yaw is the heading of that axis.

    What L's axes are at initialisation is a property of the LIO, so the prediction carries both
    readings and the measurement decides (`FRAMES.md` test T6):

      world_axes="body"    the LIO level-aligns its world with the body's heading at init
      world_axes="sensor"  the LIO's world is the housing's own frame at init, i.e. pitched 45 deg
                           down as well (Super-LIO's map_init starts from the sensor frame)
    """
    R_WL: np.ndarray          # W <- L
    t_WL: np.ndarray          # L's origin in W, metres
    n_samples: int = 0
    rms_m: float = float("nan")

    @staticmethod
    def predicted(spawn_heading_deg=0.0, spawn_pos_w=None, world_axes="sensor",
                  sensor_lever_arm_b=SENSOR_LEVER_ARM_B):
        """The transform the construction predicts, for comparison against the measurement.

        `spawn_heading_deg` is the body's **compass** heading (0 = North, +90 = East, PLAN.md
        decision 14).  L's origin is the housing's position at init, i.e. the spawn point plus the
        lever arm rotated into W.
        """
        if spawn_pos_w is None:
            spawn_pos_w = np.zeros(3)
        R_WB = R_WB_from_heading(spawn_heading_deg)
        R_WL = R_WB @ R_body_sensor() if world_axes == "sensor" else R_WB
        t = np.asarray(spawn_pos_w, dtype=float) + R_WB @ sensor_lever_arm_b
        return Calib(R_WL=R_WL, t_WL=t)

    def yaw_deg(self):
        """Yaw of R_WL: the heading, in W, of L's x axis."""
        return math.degrees(yaw_from_R(self.R_WL))

    def summary(self):
        return "L->W yaw %+.3f deg, t = [%+.4f %+.4f %+.4f] m (n=%d, rms %.4f m)" % (
            self.yaw_deg(), self.t_WL[0], self.t_WL[1], self.t_WL[2], self.n_samples, self.rms_m)


# --------------------------------------------------------------------------- self-test
def _selftest():
    ok = True

    def check(name, got, want, tol=1e-12):
        nonlocal ok
        good = np.allclose(np.asarray(got, dtype=float), np.asarray(want, dtype=float), atol=tol)
        ok = ok and good
        print("%-46s %s" % (name, "ok" if good else "FAIL got=%s want=%s" % (got, want)))

    check("ned_to_enu(1,2,3)", ned_to_enu([1, 2, 3]), [2, 1, -3])
    check("enu_to_ned round trip", enu_to_ned(ned_to_enu([1.5, -2.5, 3.5])), [1.5, -2.5, 3.5])
    check("frd_to_flu(1,2,3)", frd_to_flu([1, 2, 3]), [1, -2, -3])
    check("P determinant", np.linalg.det(P_NED_ENU), 1.0)

    # A body pitched 30 deg nose up, heading North, expressed body-FRD -> NED.
    # In FRD a nose-up pitch by theta is R_y(+theta): the body x axis, seen in NED, becomes
    # (cos, 0, -sin) -- forward and UP (NED z is down).
    R_ned_frd = R_y(math.radians(30.0))
    p_w, v_w, q_w = wire_to_world([0, 0, -2.0], [1, 0, 0], q_from_R(R_ned_frd))
    check("tap position NED(0,0,-2) -> W", p_w, [0, 0, 2])
    check("tap velocity NED north -> W", v_w, [0, 1, 0])
    R_w_b = R_from_q(q_w)
    check("tap attitude: body x -> North and 30 deg up", R_w_b @ [1, 0, 0],
          [0, math.cos(math.radians(30)), math.sin(math.radians(30))], 1e-12)
    check("tap attitude: body up leans 30 deg South", R_w_b @ [0, 0, 1],
          [0, -math.sin(math.radians(30)), math.cos(math.radians(30))], 1e-12)
    check("world_to_wire round trip", world_to_wire(p_w, v_w, q_w)[0], [0, 0, -2.0], 1e-12)

    # Spawn facing North (PLAN.md decision 14), compass heading 0, body x = W north = (0,1,0)_W.
    c_body = Calib.predicted(spawn_heading_deg=0.0, world_axes="body")
    check("spawn North, body axes: L's x is W north", c_body.R_WL[:, 0], [0, 1, 0], 1e-12)
    check("spawn North, body axes: L's y is W west", c_body.R_WL[:, 1], [-1, 0, 0], 1e-12)
    check("spawn North, body axes: yaw(R_WL)", c_body.yaw_deg(), 90.0, 1e-9)

    # With the housing's own axes, L's x is 45 deg below the horizon, pointing north (the mount is
    # a pure pitch, so it does not change the heading -- only the elevation of L's x axis).
    c_sens = Calib.predicted(spawn_heading_deg=0.0, world_axes="sensor")
    check("spawn North, sensor axes: L's x is North and 45 deg down", c_sens.R_WL[:, 0],
          [0.0, math.cos(math.radians(45)), -math.sin(math.radians(45))], 1e-12)
    check("spawn North, sensor axes: yaw(R_WL)", c_sens.yaw_deg(), 90.0, 1e-9)
    check("spawn North: L origin is the lever arm in W", c_sens.t_WL, [0.0, 0.05, 0.10], 1e-12)

    c_east = Calib.predicted(spawn_heading_deg=90.0, world_axes="body")
    check("spawn East, body axes: L's x is W east", c_east.R_WL[:, 0], [1, 0, 0], 1e-12)
    check("spawn East, body axes: yaw(R_WL)", c_east.yaw_deg(), 0.0, 1e-9)

    # The quaternion helpers must agree with the matrix exponential form for a known rotation.
    check("q_from_R(R_z(90))", q_from_R(R_z(math.pi / 2)), [math.cos(math.pi / 4), 0, 0,
                                                            math.sin(math.pi / 4)], 1e-12)
    print("selftest:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(_selftest())
