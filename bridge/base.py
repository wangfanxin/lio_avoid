#!/usr/bin/env python3
"""The transport interface: ONE shape, and at HEAD exactly ONE implementation.

Everything above this line speaks the frame `F` with ENU labels and z up (PLAN.md decision 4): the FCU's
own local frame, and the LIO's world once the origin is settled.  Everything below it speaks whatever
the implementation needs, and each implementation crosses the frame boundary exactly once:

  * `mavlink_impl` converts itself: F (ENU) -> NED for position and velocity, and it composes the
    ODOMETRY message in the frames the firmware actually accepts (`LOCAL_FRD`/`BODY_FRD`; in the other
    frames ARDUPILOT SILENTLY DROPS IT -- measured from source, FRAMES.md 0.2).
  * a `mavros_impl` would hand ENU-labelled numbers to mavros and let MAVROS do the conversion
    (`vision_pose_estimate.cpp:133-139`), and must NOT also flip them.

That asymmetry is the whole reason this file exists: it is the one place a double conversion can hide.

FINDING, MEASURED AT HEAD eba1033 -- AND THIS FILE NO LONGER REPEATS THE FALSE CLAIM.
`bridge/mavros_impl.py` **DOES NOT EXIST**.  `ls bridge/` is `base.py`, `mavlink_impl.py`, `__pycache__`.
The v1 docstring said "ONE shape, two implementations (`BRIDGE=mavlink|mavros`)", and three other places
still say so: `fsm/mission_fsm.py:152-153` imports `MavrosBridge` as the non-mavlink branch (so
`--bridge mavros` raises ModuleNotFoundError at the first call), `docs/PLAN.md:34` (decision 4) and
`docs/ARCHITECTURE.md:225` ("`mavros_impl` exists and is unflown").  The `mavros` paragraph above is
written in the conditional for that reason: it is a contract a second implementation WOULD have to meet,
not a description of a file.  Writing that file is deliberately NOT part of the boundary refactor
(refactor-bridge-plan.md section 8 item 11).  Recorded for the later documentation round in
`_stage/refactor-S2-findings.txt`.

WHAT THIS FILE IS FOR (and why it was completed on 2026-10-16).  The mission calls three methods that
this class did not declare -- `send_position_velocity` (`fsm/mission_fsm.py:388`, `:391`),
`set_ekf_origin` (`:572`), `calibrate_accel_simple` (`:582`) -- and reaches into implementation privates
for its own final log (`:972-973`, `self.bridge._sent_positions` / `._sent_nav`).  A contract that the
only caller does not obey is not a contract, so the declarations below were completed and
`tools/test-bridge-contract.py` now makes the claim enforceable: every `self.bridge.<name>` used anywhere
in the tree must resolve on `Bridge`, and deleting one declaration fails that test.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class Mode(Enum):
    GUIDED = "GUIDED"
    LAND = "LAND"
    RTL = "RTL"
    LOITER = "LOITER"
    ALT_HOLD = "ALT_HOLD"
    STABILIZE = "STABILIZE"


@dataclass
class LocalState:
    """What the FCU believes, in F (ENU labels, z up), plus its own health."""
    t_us: int = 0
    pos: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    vel: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    armed: bool = False
    mode: str = ""
    ekf_ok: bool = False
    ekf_origin_set: bool = False
    gps_fix: int = 0
    battery_v: float = 0.0
    alt_amsl_m: float = 0.0        # the FCU's own global altitude (what TAKEOFF's guard compares to)
    alt_rel_m: float = 0.0         # ... and the same relative to home
    landed_state: int = -1         # EXTENDED_SYS_STATE: 1 = on ground, 2 = in air (TAKEOFF needs 1)
    servos: list = field(default_factory=lambda: [0] * 8)   # SERVO_OUTPUT_RAW, for "is it even trying?"
    last_heartbeat_us: int = 0
    last_position_us: int = 0
    messages: list = field(default_factory=list)     # recent statustext, for the record

    def age_us(self, now_us: int) -> int:
        return now_us - self.last_position_us if self.last_position_us else 10 ** 9


class Bridge:
    """The interface the FSM is written against.  Nothing here knows about MAVLink or ROS."""

    def __init__(self, log=None):
        self.log = log or (lambda s: None)

    # ---- commands (F, ENU labels, z up) ------------------------------------------------------
    def arm(self, timeout_s: float = 20.0) -> bool:
        raise NotImplementedError

    def disarm(self) -> bool:
        raise NotImplementedError

    def set_mode(self, mode: Mode) -> bool:
        raise NotImplementedError

    def takeoff(self, alt_m: float) -> bool:
        raise NotImplementedError

    def land(self) -> bool:
        raise NotImplementedError

    def calibrate_accel_simple(self) -> bool:
        """The one-shot accelerometer calibration a fresh EEPROM needs (COMMAND_INT, x = 4).

        Declared here because the mission calls it (`fsm/mission_fsm.py:582`); it is a 76-class
        preflight AUTHORITY action, so its only caller becomes the supervisor.  The measured story of
        why it cannot be a COMMAND_LONG is in `mavlink_impl.py:174-189`.
        """
        raise NotImplementedError

    def calibrate_gyros(self) -> bool:
        """The gyro calibration; same class and same owner as `calibrate_accel_simple`."""
        raise NotImplementedError

    def set_ekf_origin(self, lat_deg: float, lon_deg: float, alt_m: float,
                       attempts: int = 12, pump=None) -> bool:
        """Declare the EKF origin (cmd 611, COMMAND_INT) and confirm it from the FCU's own broadcast.

        `pump(seconds)` keeps external nav flowing while the origin retries -- the FCU refuses the
        origin while it is still on DCM, so the retries are "wait for EKF3 to become primary", not
        flakiness (`mavlink_impl.py:268-288`).  The MESSAGE is transport and stays here; the RETRY
        POLICY is preflight sequencing and moves to the supervisor.
        """
        raise NotImplementedError

    def send_position(self, p_f, yaw_rad: float) -> None:
        """Streamed at 50 Hz by the FSM: LOCAL_NED position + yaw, the two things GUIDED accepts."""
        raise NotImplementedError

    def send_velocity(self, v_f, yaw_rad: float) -> None:
        raise NotImplementedError

    def send_position_velocity(self, p_f, v_f, yaw_rad: float, a_f=None) -> None:
        """Position AND velocity (and acceleration when given) in ONE setpoint -- msg 84, `POS_VEL_YAW`
        or `POS_VEL_ACC_YAW`.  This is the arm of `--setpoint-mode` the mission actually flies
        (`fsm/mission_fsm.py:388`, `:391`); `a_f=None` means "do not send acceleration", which is not
        the same as sending zeros (the type mask changes with it).
        """
        raise NotImplementedError

    def send_external_nav(self, t_us: int, p_f, q_f, v_f=None) -> None:
        """The LIO's pose (and velocity) in F, ENU labels, z up.  This is what makes the EKF work."""
        raise NotImplementedError

    # ---- state ------------------------------------------------------------------------------
    def state(self) -> LocalState:
        raise NotImplementedError

    def poll(self) -> None:
        """Non-blocking: drain the socket, update `state()`."""
        raise NotImplementedError

    def counters(self) -> dict:
        """The transport's own tally, for a run's final line: at least `positions` and `external_nav`.

        Declared here because the alternative is what the mission did until 2026-10-16: reading
        `bridge._sent_positions` / `bridge._sent_nav` (`fsm/mission_fsm.py:972-973`), i.e. depending on
        a private attribute of the ONE implementation -- which a second implementation would silently
        break.  The values are the same numbers (`mavlink_impl.py:416-417`).
        """
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError


def yaw_from_quat(q):
    """Yaw (about +z) of a scalar-first quaternion, radians."""
    import math
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_from_yaw(yaw):
    import math
    return [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]
