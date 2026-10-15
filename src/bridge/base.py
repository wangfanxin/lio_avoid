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

WHAT THIS FILE IS FOR (completed 2026-10-16 in S2, and made SINGLE-CONTRACT in F1).  In S2 the mission
called three methods this class did not declare -- `send_position_velocity`, `set_ekf_origin`,
`calibrate_accel_simple` -- and reached into implementation privates for its own final log; the
declarations were completed and `tools/test-bridge-contract.py` makes the claim enforceable.  F1 then
removed the reason the setpoint half was ambiguous: there were THREE setpoint writers with TWO frame
contracts (`send_position`/`send_position_velocity`/`send_velocity` took `F`/`W` numbers, `send_cmd` takes
`L`), and a caller had to know which it was calling.  All three are DELETED, with their call sites: the
plan box's command stream reaches the wire through `send_cmd` and nothing else, and the estimator input
through `send_external_nav_l` (L) or `send_external_nav` (F, the truth-fed calibration flight only).  The
frame a caller speaks is now part of every method's NAME, which is the only version of this contract that
a reader cannot get wrong.
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

    def send_external_nav(self, t_us: int, p_f, q_f, v_f=None) -> None:
        """The LIO's pose in **F** (ENU labels, z up).  This is what makes the EKF work.

        Since F1 the mission feeds this through `send_external_nav_l`; the only caller left that speaks F
        is the CALIBRATION flight (`--nav-source truth`), whose whole purpose is to measure L -> F and
        which therefore cannot use the calibration it is measuring.
        """
        raise NotImplementedError

    def send_external_nav_l(self, t_us: int, p_l, q_l, v_l, calib) -> None:
        """The LIO's pose in **L**, placed on the wire in ONE conversion (F1).

        `p_f = R_WL @ p_L + t_WL`, `q_f = q_from_R(R_WL @ R_from_q(q_L))`,
        `v_f = R_WL @ v_L` -- the expression `fsm/mission_fsm.py:465-469` used to compute, in the file
        whose docstring claims the frame boundary is crossed exactly once.  `q_l` is body-FLU -> L.
        """
        raise NotImplementedError

    def send_cmd(self, cmd_l, calib, yaw_rad=None, fields="pos"):
        """A command in **L** (`cmd_l.p` / `.v` / `.a` / `.yaw`), placed on the wire in ONE conversion:
        `permute(R_WL @ p_L + t_WL)` for the position, the rotation alone for velocity and acceleration,
        yaw passed through (it is the heading LOCAL_NED reads: 0 = North, +90 deg = East).

        This is the PLAN box's interface and THE ONLY setpoint entry point since F1: the three F-frame
        writers that coexisted with it are deleted, so there is nothing left for a caller to choose
        between.  `calib` is the SAME `common.frames.Calib` (W <- L) that the plan box and the preflight
        measurement use; the transport does not own it, it applies it.

        The assumption this makes explicit is "the EKF origin is the pad" (so `F` is `W` numerically).
        It was inherited silently at `fsm/mission_fsm.py:446-449`; here it is named.

        `fields` selects what the wire carries -- "pos" (the flown default: position + yaw),
        "posvel" or "posvelacc".  It is the old `--setpoint-mode`, and `mavlink_impl.send_cmd` records the
        measurement that decides the default.
        """
        raise NotImplementedError

    # ---- state ------------------------------------------------------------------------------
    def state(self) -> LocalState:
        raise NotImplementedError

    def poll(self) -> None:
        """Non-blocking: drain the socket, update `state()`."""
        raise NotImplementedError

    def request_rates(self) -> None:
        """Ask THIS endpoint for the streams it needs (511 MAV_CMD_SET_MESSAGE_INTERVAL), and let a
        consumer ask AGAIN.

        Declared because a caller must be able to re-ask: MEASURED on the F2 supervisor's own endpoint, a
        511 request issued while the FCU is still booting delivers 0 Hz forever after, and the F2 bench
        measured the same shape from the other side (asked 50 -> 50.2 Hz, asked 30 -> 25.3, asked 20 -> 0.0).
        The rule the two measurements produce is "request, MEASURE, and keep asking until it arrives" --
        which needs this to be public rather than a constructor side effect.
        """
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
