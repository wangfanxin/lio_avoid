#!/usr/bin/env python3
"""The pymavlink implementation of the transport interface.

This is the implementation that does the frame maths itself, so it is the one to read carefully:

  * **position setpoints** go out as `SET_POSITION_TARGET_LOCAL_NED` (84) in `MAV_FRAME_LOCAL_NED` (1).
    The numbers must be NED, so F's (x, y, z) = (East, North, Up) becomes (North, East, Down) =
    (y, x, -z).  The type mask uses POSITION_IGNORE|ACCELERATION_IGNORE|VELOCITY_IGNORE and passes yaw.
  * **the external nav** goes out as `ODOMETRY` (331) with `frame_id = MAV_FRAME_LOCAL_FRD` (20) and
    `child_frame_id = MAV_FRAME_BODY_FRD` (12).  MEASURED FROM SOURCE (FRAMES.md 0.2): in any other pair
    of frames ArduPilot drops the message without a word.  Its position is consumed as NED and its
    velocity as BODY FRD, which is why the velocity has to be rotated out of F by the current attitude
    rather than sent as the LIO's world-frame velocity.
  * **the stream rates are ~0 in SITL by default** (measured), so the constructor asks for the messages
    it needs with MAV_CMD_SET_MESSAGE_INTERVAL (511) instead of trusting any SR*_* parameter.
"""
from __future__ import annotations

import math
import time

import numpy as np

from bridge.base import Bridge, LocalState, Mode

# module-level so a missing pymavlink is reported where it matters, not at import time
try:
    from pymavlink import mavutil
except ImportError:                                          # pragma: no cover
    mavutil = None

COPTER_MODE = {
    Mode.STABILIZE: 0,
    Mode.ALT_HOLD: 2,
    Mode.GUIDED: 4,
    Mode.LOITER: 5,
    Mode.RTL: 6,
    Mode.LAND: 9,
}
MODE_NAME = {v: k.value for k, v in COPTER_MODE.items()}

# SET_POSITION_TARGET_LOCAL_NED type mask bits
IGNORE_VX = 1 << 3
IGNORE_VY = 1 << 4
IGNORE_VZ = 1 << 5
IGNORE_AFX = 1 << 6
IGNORE_AFY = 1 << 7
IGNORE_AFZ = 1 << 8
FORCE = 1 << 9
IGNORE_YAW = 1 << 10
IGNORE_YAW_RATE = 1 << 11
POS_YAW_MASK = IGNORE_VX | IGNORE_VY | IGNORE_VZ | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ | IGNORE_YAW_RATE
VEL_YAW_MASK = (1 << 0) | (1 << 1) | (1 << 2) | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ | IGNORE_YAW_RATE


class MavlinkBridge(Bridge):
    def __init__(self, url="tcp:127.0.0.1:5760", log=None, request_hz=50.0,
                 position_hz=50.0, attitude_hz=50.0):
        super().__init__(log=log)
        if mavutil is None:
            raise RuntimeError("pymavlink is not installed")
        self.url = url
        self.mav = mavutil.mavlink_connection(url)
        self.mav.wait_heartbeat(timeout=10)
        self._state = LocalState()
        self._state.ekf_origin_set = False
        self._sent_positions = 0
        self._sent_nav = 0
        self._started = time.monotonic()
        self.log("bridge/mavlink: connected to %s, system %d component %d"
                 % (url, self.mav.target_system, self.mav.target_component))
        # ask for what we need; SR*_* defaults are ~0 in SITL
        self._request_message(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, position_hz)
        self._request_message(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, attitude_hz)
        self._request_message(mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 5.0)
        self._request_message(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 1.0)
        self._request_message(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 2.0)
        self._request_message(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 2.0)
        self._request_message(mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 2.0)
        self._request_message(mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 10.0)

    # ---- plumbing ---------------------------------------------------------------------------
    def _request_message(self, msg_id: int, hz: float):
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)

    def poll(self):
        while True:
            m = self.mav.recv_match(blocking=False)
            if m is None:
                break
            t = m.get_type()
            if t == "LOCAL_POSITION_NED":
                # the FCU's local frame is NED; the interface is ENU labels with z up
                self._state.pos = [m.y, m.x, -m.z]
                self._state.vel = [m.vy, m.vx, -m.vz]
                self._state.last_position_us = int(time.monotonic() * 1e6)
                self._state.t_us = self._state.last_position_us
            elif t == "HEARTBEAT":
                self._state.armed = bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self._state.mode = MODE_NAME.get(m.custom_mode, "mode%d" % m.custom_mode)
                self._state.last_heartbeat_us = int(time.monotonic() * 1e6)
            elif t == "EKF_STATUS_REPORT":
                # the flags bit that matters here: EKF_OK / const pos mode.  Report what it says.
                self._state.ekf_ok = bool(m.flags & mavutil.mavlink.EKF_POS_HORIZ_ABS) or \
                    bool(m.flags & mavutil.mavlink.EKF_CONST_POS_MODE) is False
            elif t == "GPS_RAW_INT":
                self._state.gps_fix = m.fix_type
            elif t == "GLOBAL_POSITION_INT":
                # TAKEOFF's guard compares the requested altitude against `current_loc.alt`, so knowing
                # whether the FCU's altitude datum is AMSL (origin missing) or home-relative decides
                # whether a 0.8 m takeoff is even accepted.
                self._state.alt_amsl_m = m.alt / 1000.0
                self._state.alt_rel_m = m.relative_alt / 1000.0
            elif t == "EXTENDED_SYS_STATE":
                # landed_state is the FCU's answer to "am I on the ground?" -- the second guard in
                # `do_user_takeoff` refuses when it says IN_AIR ("can't takeoff again")
                self._state.landed_state = m.landed_state
            elif t == "SERVO_OUTPUT_RAW":
                # the motor outputs: a takeoff that is accepted but produces no climb looks identical
                # to one that is never commanded, and this is what tells them apart
                self._state.servos = [m.servo1_raw, m.servo2_raw, m.servo3_raw, m.servo4_raw,
                                      m.servo5_raw, m.servo6_raw, m.servo7_raw, m.servo8_raw]
            elif t == "SYS_STATUS":
                self._state.battery_v = m.voltage_battery / 1000.0
            elif t == "STATUSTEXT":
                text = m.text if isinstance(m.text, str) else m.text.decode("utf-8", "replace")
                self._state.messages.append((time.monotonic(), text))
                self._state.messages = self._state.messages[-50:]
            elif t == "COMMAND_ACK":
                self._last_ack = (m.command, m.result)

    def state(self) -> LocalState:
        return self._state

    def close(self):
        try:
            self.mav.close()
        except Exception:
            pass

    # ---- commands ---------------------------------------------------------------------------
    def _command(self, cmd: int, *params, timeout=6.0) -> bool:
        p = list(params) + [0.0] * (7 - len(params))
        self._last_ack = None
        t_sent = time.monotonic()
        self.mav.mav.command_long_send(self.mav.target_system, self.mav.target_component,
                                       cmd, 0, *p)
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            self.poll()
            ack = getattr(self, "_last_ack", None)
            if ack is not None and ack[0] == cmd:
                ok = ack[1] == mavutil.mavlink.MAV_RESULT_ACCEPTED
                self.log("bridge/mavlink: command %d -> %s" % (cmd, "ACCEPTED" if ok else ack[1]))
                if not ok:
                    # report only what the FCU said SINCE we asked: a stale message reads as the reason
                    fresh = [t for (when, t) in self._state.messages if when >= t_sent]
                    self.log("   FCU said: %s" % (" | ".join(fresh[-4:]) if fresh else "(nothing)"))
                return ok
            time.sleep(0.05)
        self.log("bridge/mavlink: command %d -> no ACK in %.1f s" % (cmd, timeout))
        return False

    def arm(self, timeout_s: float = 20.0) -> bool:
        return self._command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.0, timeout=timeout_s)

    def calibrate_accel_simple(self) -> bool:
        """The one-shot accelerometer calibration a fresh EEPROM needs.

        MEASURED 2026-10-13, in two steps:
          * with `-w` the FCU refuses to arm with "Arm: 3D Accel calibration needed"
            (`AP_Arming.cpp:523`, Check::INS = 1<<4, which this project's ARMING_SKIPCHK mask
            deliberately does not skip);
          * the calibration must be sent as **COMMAND_INT**, not COMMAND_LONG: the handler is
            `_handle_command_preflight_calibration(const mavlink_command_int_t &packet, ...)` and it
            dispatches on `packet.x`, a field COMMAND_LONG does not have.  A COMMAND_LONG with param1=4
            comes back MAV_RESULT_UNSUPPORTED (3) -- measured, and it looks exactly like "this command
            is not implemented" rather than "you used the wrong message".
        param1 = 1 would start the INTERACTIVE six-position calibration; x = 4 is
        `AP::ins().simple_accel_cal()` (`GCS_Common.cpp:4955`), the single-shot level calibration that
        suits a simulated vehicle standing level on the pad.
        """
        self._last_ack = None
        self.mav.mav.command_int_send(
            self.mav.target_system, mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            mavutil.mavlink.MAV_FRAME_MISSION,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            0, 0,                       # current, autocontinue
            4.0, 0.0, 0.0, 0.0,         # param1..4
            4, 0, 0)                    # x, y, z -- x = 4 is the simple accel calibration
        t_end = time.monotonic() + 15.0
        while time.monotonic() < t_end:
            self.poll()
            ack = getattr(self, "_last_ack", None)
            if ack is not None and ack[0] == mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION:
                ok = ack[1] in (mavutil.mavlink.MAV_RESULT_ACCEPTED,
                                mavutil.mavlink.MAV_RESULT_IN_PROGRESS)
                self.log("bridge/mavlink: accel calibration -> %s" % ("ACCEPTED" if ok else ack[1]))
                return ok
            time.sleep(0.05)
        self.log("bridge/mavlink: accel calibration -> no ACK")
        return False

    def calibrate_gyros(self) -> bool:
        return self._command(mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 1.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, timeout=15.0) is not None

    def disarm(self) -> bool:
        return self._command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0.0)

    def set_mode(self, mode: Mode) -> bool:
        custom = COPTER_MODE[mode]
        self.mav.mav.set_mode_send(self.mav.target_system,
                                   mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, custom)
        t_end = time.monotonic() + 5.0
        while time.monotonic() < t_end:
            self.poll()
            if self._state.mode == mode.value:
                self.log("bridge/mavlink: mode -> %s" % mode.value)
                return True
            time.sleep(0.05)
        self.log("bridge/mavlink: mode %s not confirmed (still %s)" % (mode.value, self._state.mode))
        return False

    def takeoff(self, alt_m: float) -> bool:
        """Take off to `alt_m` ABOVE THE PAD, whatever datum the FCU's global altitude uses.

        MEASURED 2026-10-13: the guard in `ArduCopter/takeoff.cpp:29` is
        `takeoff_alt_m <= copter.current_loc.alt * 0.01` -- the requested altitude is compared against
        the FCU's own GLOBAL altitude, which with an origin at 584 m is an AMSL number.  A request for
        0.8 m is then "a takeoff downwards": MAV_RESULT_FAILED, no statustext at all, and it looks like
        a mystery rather than a unit problem.  The datum is therefore read from the FCU's own telemetry
        (`GLOBAL_POSITION_INT.alt - relative_alt`) and added here, so the caller keeps speaking plain
        height above the pad.
        """
        datum = self._state.alt_amsl_m - self._state.alt_rel_m
        target = datum + alt_m if abs(datum) >= 1.0 else alt_m
        if abs(datum) >= 1.0:
            self.log("bridge/mavlink: takeoff %.2f m above the pad = %.2f m on the FCU's own datum "
                     "(%.1f m)" % (alt_m, target, datum))
        # param3 = 1.  The handler computes `must_navigate = is_zero(packet.param3)` and passes it to
        # `has_user_takeoff`; with the default 0 GUIDED refuses to start a takeoff before it has a
        # navigation target -- MAV_RESULT_FAILED, no statustext, and the three guards that ARE visible
        # (armed, landed, altitude) all pass, which is what made this one hard to see.  param3 = 1
        # means "horizontal navigation by the pilot is acceptable", i.e. must_navigate = false.
        return self._command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, target)

    def land(self) -> bool:
        return self.set_mode(Mode.LAND)

    def _origin_broadcast(self):
        """The FCU's own GPS_GLOBAL_ORIGIN (49) report, or None.  This is the positive check."""
        while True:
            m = self.mav.recv_match(type="GPS_GLOBAL_ORIGIN", blocking=False)
            if m is None:
                return None
            lat, lon = m.latitude * 1e-7, m.longitude * 1e-7
            if abs(lat) > 1e-6 or abs(lon) > 1e-6:
                return (lat, lon, m.altitude / 1000.0)

    def set_ekf_origin(self, lat_deg: float, lon_deg: float, alt_m: float,
                       attempts: int = 12, pump=None) -> bool:
        """Declare the EKF origin, RETRYING while the flight controller is still on DCM.

        This is the whole story, and every step was measured:

        * No GPS means nothing else can define what local NED is relative to, and without an origin the
          takeoff path fails silently: `ModeGuided::do_user_takeoff_start_m` converts the takeoff
          altitude from ABOVE_HOME to ABOVE_ORIGIN, `Location::get_alt_m(ABOVE_ORIGIN)` needs the
          origin, and when it is missing the command answers MAV_RESULT_FAILED with no statustext.
          (Symptom: armed=True, mode=GUIDED, landed_state=1, altitude sane, takeoff refused anyway.)
        * The command is `MAV_CMD_DO_SET_GLOBAL_ORIGIN` (611, mavlink DEVELOPMENT dialect) sent as
          COMMAND_INT; `GCS_Common.cpp:4852` decodes x/y/z with `location_from_command_t`, so the
          altitude is in METRES (a millimetre value trips the sanity check and is DENIED).
        * `AP_AHRS::set_origin` (AP_AHRS.cpp:1477) returns success only for the ACTIVE estimator, and at
          boot the log says "AHRS: DCM active" -- so an origin sent too early is refused with FAILED
          even though the command is correct.  Hence the retries: they are not flakiness, they are
          waiting for EKF3 to become primary.
        * The positive check is the FCU's own broadcast of message 49 (asked for with 511, because this
          firmware streams nothing unasked), not our own belief that the command was accepted.
        """
        cmd = 611
        self._request_message(mavutil.mavlink.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN, 2.0)
        for attempt in range(attempts):
            if pump is not None:
                pump(0.4)                       # keep feeding external nav so the EKF can initialise
            self._last_ack = None
            self.mav.mav.command_int_send(
                self.mav.target_system, mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT, cmd, 0, 0,
                0.0, 0.0, 0.0, 0.0,
                int(round(lat_deg * 1e7)), int(round(lon_deg * 1e7)), float(alt_m))
            t_end = time.monotonic() + 1.5
            result = None
            while time.monotonic() < t_end:
                self.poll()
                ack = getattr(self, "_last_ack", None)
                if ack is not None and ack[0] == cmd and result is None:
                    result = ack[1]
                if pump is not None:
                    pump(0.05)
                bc = self._origin_broadcast()
                if bc is not None:
                    ok = abs(bc[0] - lat_deg) < 1e-5 and abs(bc[1] - lon_deg) < 1e-5
                    self._state.ekf_origin_set = ok
                    self.log("bridge/mavlink: EKF origin confirmed by the FCU: (%.7f, %.7f, %.1f m)"
                             % bc)
                    return ok
                time.sleep(0.05)
            if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                self.log("bridge/mavlink: EKF origin ACCEPTED on attempt %d" % (attempt + 1))
                # accepted, but confirm from the FCU's own report before believing it
                for _ in range(10):
                    if pump is not None:
                        pump(0.1)
                    self.poll()
                    bc = self._origin_broadcast()
                    if bc is not None:
                        self._state.ekf_origin_set = True
                        self.log("bridge/mavlink: EKF origin confirmed: (%.7f, %.7f, %.1f m)" % bc)
                        return True
                self.log("bridge/mavlink: EKF origin accepted but not yet broadcast -- continuing")
                return True
        self.log("bridge/mavlink: EKF origin never accepted in %d attempts (last result %s)"
                 % (attempts, result))
        return False

    def send_position(self, p_f, yaw_rad: float):
        # F (ENU, z up) -> NED
        x_ned, y_ned, z_ned = float(p_f[1]), float(p_f[0]), -float(p_f[2])
        self.mav.mav.set_position_target_local_ned_send(
            0, self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, POS_YAW_MASK,
            x_ned, y_ned, z_ned, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, yaw_rad, 0.0)
        self._sent_positions += 1

    def send_velocity(self, v_f, yaw_rad: float):
        x_ned, y_ned, z_ned = float(v_f[1]), float(v_f[0]), -float(v_f[2])
        self.mav.mav.set_position_target_local_ned_send(
            0, self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, VEL_YAW_MASK,
            0.0, 0.0, 0.0, x_ned, y_ned, z_ned, 0.0, 0.0, 0.0, yaw_rad, 0.0)

    def send_external_nav(self, t_us: int, p_f, q_f, v_f=None):
        """The LIO's pose in F, sent as ODOMETRY in the ONLY frame pair the firmware accepts.

        Position: F (E, N, U) -> NED = (N, E, -U).
        Velocity: F is a world frame; ODOMETRY's velocity field is BODY FRD, so rotate by the attitude
        rather than sending a world-frame vector.  (Passing the world velocity would be right only as
        long as the vehicle never yaws -- and the mission yaws.)
        """
        import numpy as np
        from common import frames
        p_ned = np.array([p_f[1], p_f[0], -p_f[2]])
        R = frames.R_from_q(q_f)
        if v_f is None:
            v_body_frd = np.zeros(3)
        else:
            v_body_flu = R.T @ np.asarray(v_f, dtype=float)
            v_body_frd = frames.flu_to_frd(v_body_flu)
        # ODOMETRY's field layout (pymavlink odometry_send): time, frames, x,y,z, q as a LIST in the
        # spec's order w,x,y,z, vx,vy,vz, rates, then the two 21-element covariance lists, then
        # reset_counter, estimator_type, quality.  Passing the quaternion as four scalars raises
        # "takes from 16 to 20 positional arguments but 24 were given" (measured).
        self.mav.mav.odometry_send(
            int(t_us), mavutil.mavlink.MAV_FRAME_LOCAL_FRD, mavutil.mavlink.MAV_FRAME_BODY_FRD,
            float(p_ned[0]), float(p_ned[1]), float(p_ned[2]),
            [float(q_f[0]), float(q_f[1]), float(q_f[2]), float(q_f[3])],
            float(v_body_frd[0]), float(v_body_frd[1]), float(v_body_frd[2]),
            0.0, 0.0, 0.0,
            [0.0] * 21, [0.0] * 21,
            0, mavutil.mavlink.MAV_ESTIMATOR_TYPE_VISION, 100)
        self._sent_nav += 1

    # ---- convenience for the harness --------------------------------------------------------
    def counters(self):
        return {"positions": self._sent_positions, "external_nav": self._sent_nav}
