#!/usr/bin/env python3
"""The transport interface: ONE shape, two implementations (`BRIDGE=mavlink|mavros`).

Everything above this line speaks the frame `F` with ENU labels and z up (PLAN.md decision 4): the FCU's
own local frame, and the LIO's world once the origin is settled.  Everything below it speaks whatever
the implementation needs, and each implementation crosses the frame boundary exactly once:

  * `mavlink_impl` converts itself: F (ENU) -> NED for position and velocity, and it composes the
    ODOMETRY message in the frames the firmware actually accepts (`LOCAL_FRD`/`BODY_FRD`; in the other
    frames ARDUPILOT SILENTLY DROPS IT -- measured from source, FRAMES.md 0.2).
  * `mavros_impl` hands ENU-labelled numbers to mavros and lets MAVROS do the conversion
    (`vision_pose_estimate.cpp:133-139`), and must NOT also flip them.

That asymmetry is the whole reason this file exists: it is the one place a double conversion can hide.
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

    def send_position(self, p_f, yaw_rad: float) -> None:
        """Streamed at 50 Hz by the FSM: LOCAL_NED position + yaw, the two things GUIDED accepts."""
        raise NotImplementedError

    def send_velocity(self, v_f, yaw_rad: float) -> None:
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
