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
# position + velocity + yaw: use the first three axes, ignore acceleration and yaw rate
POS_VEL_YAW_MASK = IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ | IGNORE_YAW_RATE
# position + velocity + acceleration + yaw: everything the trajectory can feed forward
POS_VEL_ACC_YAW_MASK = IGNORE_YAW_RATE


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
        self.request_hz = request_hz
        self.position_hz = position_hz
        self.attitude_hz = attitude_hz
        self.request_rates()

    def request_rates(self):
        """Ask THIS link for the messages it needs (511), and let a caller ask AGAIN.

        WHY IT IS PUBLIC AND RE-ISSUABLE.  MEASURED, twice, on two different links: a 511 request issued
        while the FCU is still booting is accepted by nobody and nothing arrives -- the bench's second
        channel (serial1) received LOCAL_POSITION_NED at 50.2 Hz once it asked AFTER the first channel's
        preflight had run, and the F2 supervisor's own endpoint measured **0.0 Hz** after asking at t=0.6 s
        of its own life (`logs/missionA-RECT25-kill-plan/supervisor.log`).  A transport that can only ask
        once is a transport that silently gets nothing, so the request is a method and the CONSUMER
        verifies the result and re-asks -- which is the same discipline the second-endpoint bench enforces.
        """
        # ask for what we need; SR*_* defaults are ~0 in SITL
        self._request_message(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, self.position_hz)
        self._request_message(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, self.attitude_hz)
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
                # GOAL-TRACE/2 A FAILSAFE MUST LEAVE A TRACE WHEN IT HAPPENS.  DIAG29's whole cause
                # (EKF Failsafe | EKF variance) survived as ONE teardown line at supervisor.py:676,
                # and a grep of mine truncated even that (STATUS rows 119-120).  Knowledge-free on
                # purpose: print the branch's own scope, so the text is captured whatever it is called.
                print('FCU-TEXT %.2f %s' % (time.time(),
                      {k: v for k, v in locals().items() if k not in ('self', 't')}), flush=True)
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
        """Take off to `alt_m` ABOVE THE PAD, sent VERBATIM as `packet.z` (SETTLED BY BENCH 2026-10-15).

        THE DEFECT THIS REPLACES.  Until this commit the request was sent as `datum + alt_m` (2.50 m
        became 586.59 m with the origin at 584 m), on the reading that the guard in
        `ArduCopter/takeoff.cpp:29` -- `takeoff_alt_m <= copter.current_loc.alt * 0.01` -- compares the
        request against an AMSL-scale number, so that a small height-above-the-pad request is "a takeoff
        downwards".  The guard reading was wrong and the compensation was the bug: the SAME number is
        handed to the executor, `Mode::_TakeOff::start_m`, which computes
        `take_off_complete_alt_m = get_pos_estimate_U_m() + alt_m` (`takeoff.cpp:51-57`) -- it ADDS what
        it is given.  So `datum + alt_m` made the FCU aim at ~586 m above home.  On `RECT25-kill-plan`
        the FCU's own `CTUN.DAlt` ramped at 2 m/s from +0.215 to +9.874 m on a 2.50 m command; the
        aircraft climbed to 9.6 m and came back to 2.0 m only when the plan's setpoints arrived.

        THE OLD MEASUREMENT'S CONDITION, AND WHY IT DIFFERED.  The docstring here used to record a
        2026-10-13 measurement: a request for 0.8 m -> MAV_RESULT_FAILED with no statustext.  That
        measurement was taken BEFORE the EKF origin existed (`set_ekf_origin` did not exist yet).  With
        no origin, `ModeGuided::do_user_takeoff_start_m` cannot convert the altitude from ABOVE_HOME to
        ABOVE_ORIGIN and the command fails -- which is the failure this class now documents at
        `set_ekf_origin`, and it has nothing to do with the requested number.  The 2026-10-13 reading
        was a real failure attributed to the wrong cause.

        THE BENCH THAT SETTLED IT (`_stage/f2b-takeoff-bench.sh clean 2.5`, rc=0, SITL on the mission's
        own parameter set with the EKF origin set first, `tools/test-takeoff-contract.py --variant
        clean`), the FCU's own words and its own log:
            [12.01] datum (alt_amsl - alt_rel) = 584.019 m;  FCU local pos = [+0.000 -0.000 +0.044] F
            [12.01] MAV_CMD_NAV_TAKEOFF packet.z = 2.500 m  (the requested height, verbatim)
            [12.07] COMMAND_ACK for MAV_CMD_NAV_TAKEOFF: result 0 (ACCEPTED)
            [14.08] AIRBORNE: truth z +0.366 m, FCU U +0.356 m, landed_state 3, 2.06 s after the command
            [26.16] watch done: truth peak +2.713 m, FCU U peak +2.682 m, setpoints sent 0
            CTUN: Alt peak +2.684 m; DAlt peak +2.525 m
            ACCEPTANCE: |adopted DAlt - request| = 0.055 m (bound 1.00 m); overshoot +0.025 m
        So the guard does NOT need a datum-scale number: `current_loc.alt` is itself alt-above-home
        (`ArduCopter/inertia.cpp:24-30` sets exactly that), `* 0.01` makes the comparison
        metres-against-metres, and on the pad the threshold is ~0.002 m -- 2.50 passes it with room to
        spare.  The executor's contract (alt-above-home, `mode_guided.cpp:157-188`) is the guard's
        contract, and it is the caller's contract too, so the request goes out verbatim.

        The datum is still COMPUTED AND LOGGED, because it is the number that used to be sent: a
        regression would otherwise be invisible in the log until the aircraft was at 9 m.
        """
        datum = self._state.alt_amsl_m - self._state.alt_rel_m
        target = alt_m
        if abs(datum) >= 1.0:
            self.log("bridge/mavlink: takeoff %.2f m above the pad sent VERBATIM as packet.z; the FCU "
                     "datum (alt_amsl - alt_rel) is %.1f m and is NOT added (adding it was the defect: "
                     "the executor ADDS packet.z to its own up-estimate, takeoff.cpp:51-57)"
                     % (alt_m, datum))
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

    # ---- THE ONE SETPOINT ENTRY POINT: L IN, WIRE OUT (S4, and the ONE entry point since F1) ---------
    #
    # WHAT DIED HERE IN F1 (2026-10-16), and why nothing lost a capability.  Until F1 this class had
    # THREE setpoint writers with TWO frame contracts, and a caller had to know which one it was calling:
    #
    #   send_position(p_f, yaw)               F/W numbers, POS_YAW mask   <- the corner tour flew THIS one
    #   send_position_velocity(p_f, v_f, yaw) F/W numbers, POS_VEL_YAW     <- `--setpoint-mode posvel` arm
    #   send_velocity(v_f, yaw)               F/W numbers, VEL_YAW         <- `--follow path` vel mode only
    #
    # Measured at HEAD 940cb50: `send_position` was the ONLY one the flown corner tour ever reached
    # (`fsm/mission_fsm.py::pump` passes `setpoint` and no `velocity`, so its `posvel` default fell through
    # to `send_position`), `send_position_velocity` had NO caller at all, and `send_velocity` had exactly
    # one -- inside `--follow path`, which F1 deleted with the Path.  The plan box's stream now reaches the
    # wire through `send_cmd` and nothing else, which is the whole point of the step: ONE conversion, ONE
    # contract, and the frame is L at every call site above this line.
    #
    # `fields` IS the old `--setpoint-mode`, kept as an explicit argument rather than a default-free
    # guess, because the mask is a FLOWN-CONFIGURATION decision and this project has a measurement for
    # it: MEASURED 2026-10-14 (sortie G2) the POS_VEL_ACC_YAW mask with an acceleration of ZERO sent at the
    # plan's end let the aircraft fly on at 2.6 m/s for 2.6 m until the geofence aborted it, while the same
    # mission on POS_YAW stopped 0.020-0.023 m from the commanded point.  So "posvelacc" is available,
    # documented, and NOT the default.
    SETPOINT_FIELDS = ("pos", "posvel", "posvelacc")

    def send_cmd(self, cmd_l, calib, yaw_rad=None, fields="pos"):
        """A `PositionCommand`-shaped command in **L**, placed on the wire in ONE conversion.

        WHAT MOVED HERE, AND WHY IT IS THE SAME ARITHMETIC.  Until 2026-10-16 the L -> W step lived in
        the mission (`fsm/mission_fsm.py:446-449` built what it called `p_f`) and the W -> NED permutation
        lived here.  `F` is `W` numerically -- the EKF origin is set to the pad -- so the two steps
        composed correctly, but the transport was silently depending on that identity while
        `docs/ARCHITECTURE.md:206` and this file's `base.py` both claim the transport crosses the frame
        boundary exactly once.  The expression the chain computes is `permute(R_WL @ p_L + t_WL)`, which is
        what this method computes, in this file, once.  The A/B that proves it is bit-identical (not merely
        close) is `tools/test-s4-l-to-wire.py`, which still compares against a written-out copy of the
        deleted entry points.

        FIELDS.  `p` crosses as `frames.l_to_wire`; `v` and `a` cross by the rotation alone
        (`frames.l_to_wire_velocity`: a velocity is a difference of positions, so `t_WL` cancels and must
        NOT be added).  `yaw` is passed through (`yaw_rad=None` means "take it from the command") -- it is
        the heading LOCAL_NED reads (0 = North, +90 deg = East), the convention
        `planner/cmd_stream.py` states at the top of the file.

        `fields` selects which of the three the WIRE carries, and it is the old `--setpoint-mode`:

          "pos"        position + yaw, the rest ignored -- **the default, and what every mission-A sortie
                       actually flew** (see the block comment above: the FSM's posvel arm fell through to
                       `send_position`, and the POS_VEL_ACC arm is the one measured to break holding);
          "posvel"     position + velocity + yaw (a source with a time parameterisation, e.g. ego);
          "posvelacc"  everything the trajectory can feed forward.

        `cmd_l` is anything with `p`, `v`, `a` and `yaw` (a `planner.cmd_stream.CmdSample`, or a
        `quadrotor_msgs` message converted to one).  Returns the W point it sent, for a test's benefit.
        """
        from common import frames
        if fields not in self.SETPOINT_FIELDS:
            raise ValueError("send_cmd: fields must be one of %s, not %r"
                             % (", ".join(self.SETPOINT_FIELDS), fields))
        p_f = frames.l_to_wire(cmd_l.p, calib)
        yaw = float(cmd_l.yaw if yaw_rad is None else yaw_rad)
        x_ned, y_ned, z_ned = float(p_f[1]), float(p_f[0]), -float(p_f[2])
        if fields == "pos":
            # LITERAL zeros, not the command's (zero) velocity: the mask says the FCU ignores them, and
            # `-0.0` is a different byte from `0.0` to the zero-tolerance A/B this method is proven with.
            vx = vy = vz = ax = ay = az = 0.0
            mask = POS_YAW_MASK
        else:
            v_f = frames.l_to_wire_velocity(cmd_l.v, calib)
            vx, vy, vz = float(v_f[1]), float(v_f[0]), -float(v_f[2])
            if fields == "posvel":
                ax = ay = az = 0.0
                mask = POS_VEL_YAW_MASK
            else:
                a_f = frames.l_to_wire_velocity(cmd_l.a, calib)
                ax, ay, az = float(a_f[1]), float(a_f[0]), -float(a_f[2])
                mask = POS_VEL_ACC_YAW_MASK
        self.mav.mav.set_position_target_local_ned_send(
            0, self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
            x_ned, y_ned, z_ned, vx, vy, vz, ax, ay, az, yaw, 0.0)
        self._sent_positions += 1
        return p_f

    def send_external_nav_l(self, t_us: int, p_l, q_l, v_l, calib):
        """The LIO's pose in **L**, fed to the FCU in ONE conversion -- the estimator path's L -> wire.

        The counterpart of `send_cmd` on the other side of the bridge: the mission used to do this
        arithmetic itself (`fsm/mission_fsm.py:465-469`: `p_f = R @ p_l + t`, `q_f = q_from_R(R @ R_from_q(q))`,
        `v_f = R @ v`) and this method is that same expression, composed here so that NOTHING above the
        bridge converts a frame any more.  `q_l` is body-FLU -> L (the LIO's own convention);
        `send_external_nav` then does the wire labelling it always did.
        """
        from common import frames
        p_f = frames.l_to_wire(p_l, calib)
        q_f = frames.q_from_R(calib.R_WL @ frames.R_from_q(q_l))
        v_f = None if v_l is None else frames.l_to_wire_velocity(v_l, calib)
        self.send_external_nav(t_us, p_f, q_f, v_f)

    def send_external_nav(self, t_us: int, p_f, q_f, v_f=None):
        """The LIO's pose in F (ENU labels, z up), sent as ODOMETRY in the frame pair the firmware reads.

        EVERY field is converted here, and the quaternion is the one that cost a flight:

          position: F (E, N, U) -> NED = (N, E, -U)
          attitude: F is body-FLU -> ENU; ODOMETRY wants body-FRD -> NED.  MEASURED 2026-10-13: sending
            q_f unconverted made the FCU believe a North-facing vehicle faced EAST.  The position loop
            then corrected sideways, the aircraft ran away ~10 m, the EKF failsafe fired ("EKF Failsafe:
            changed to Land Mode") and it hit the ground at 0.56 m/s.  Nothing in the telemetry said
            "your quaternion is in the wrong frame": XKF1 simply showed yaw +90 while roll and pitch
            were fine, which looks like a yaw-estimation problem rather than a conversion one.
          velocity: F is a world frame; the message's velocity field is BODY FRD, so it is rotated by
            the (now correct) attitude, not sent as a world vector.
        """
        import numpy as np
        from common import frames
        # ONE call crosses the frame boundary, and it converts position, velocity and attitude together
        p_ned, v_ned, q_ned = frames.world_to_wire(p_f, v_f if v_f is not None else (0.0, 0.0, 0.0),
                                                  q_f)
        if v_f is None:
            v_body_frd = np.zeros(3)
        else:
            # the message carries a BODY-FRD velocity, so rotate the world velocity by the attitude
            # the message itself carries
            v_body_frd = frames.R_from_q(q_ned).T @ np.asarray(v_ned, dtype=float)
        # ODOMETRY's field layout (pymavlink odometry_send): time, frames, x,y,z, q as a LIST in the
        # spec's order w,x,y,z, vx,vy,vz, rates, then the two 21-element covariance lists, then
        # reset_counter, estimator_type, quality.  Passing the quaternion as four scalars raises
        # "takes from 16 to 20 positional arguments but 24 were given" (measured).
        self.mav.mav.odometry_send(
            int(t_us), mavutil.mavlink.MAV_FRAME_LOCAL_FRD, mavutil.mavlink.MAV_FRAME_BODY_FRD,
            float(p_ned[0]), float(p_ned[1]), float(p_ned[2]),
            [float(q_ned[0]), float(q_ned[1]), float(q_ned[2]), float(q_ned[3])],
            float(v_body_frd[0]), float(v_body_frd[1]), float(v_body_frd[2]),
            0.0, 0.0, 0.0,
            [0.0] * 21, [0.0] * 21,
            0, mavutil.mavlink.MAV_ESTIMATOR_TYPE_VISION, 100)
        self._sent_nav += 1

    # ---- convenience for the harness --------------------------------------------------------
    def counters(self):
        return {"positions": self._sent_positions, "external_nav": self._sent_nav}
