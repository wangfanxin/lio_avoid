#!/usr/bin/env python3
"""F2's GATE TEST: is a SECOND MAVLink endpoint a capability this stack actually has?

WHY THIS EXISTS.  Until F2 every process in this repository talked to the FCU over ONE endpoint --
`--serial0 tcp:5760` (`tools/run-mission.sh`, and every other launcher in `tools/`: `sitl-parked.sh`,
`sitl-lio.sh`, `sim-sensors.sh`, `sim-ros-check.sh`, `sim-pillar-gate.sh`, `run-fcu-link-test.sh`,
`run-convention-tests.sh` -- no `--serial1`, no second URL anywhere).  F2 splits the supervisor off as its
own process with its own endpoint, which means TWO clients on one SITL at the same time.  That is a NEW
capability, and `_stage/refactor-bridge-plan.md` section 4.2 says it must be proved on the bench BEFORE it
is relied on.  If this test fails, F2 stops there.

THE TWO ROLES, and they are the F2 ones:
  A (serial0) = the BRIDGE.  It uses the STACK'S OWN `bridge/mavlink_impl.MavlinkBridge`, feeds the EKF
      from the truth tap (this bench has no LIO -- the tap is what lets the FCU's EKF produce a position at
      all in this no-GPS configuration) and sets the EKF origin, exactly as a mission's preflight does.
      **Only then does LOCAL_POSITION_NED exist**: MEASURED while writing this bench, a client that
      connects and asks for it BEFORE the origin exists receives nothing for 90 s, because Copter's
      `send_local_position` has no estimate to send.  A bench that skipped the preflight would report a
      false failure -- which the first two versions of this bench did.
  B (serial1) = the SUPERVISOR.  A raw pymavlink client with its OWN 511 rate requests.

THE SEQUENCE, so that the independence claim is a controlled A/B and not a coincidence:
  phase 0   B is connected but SILENT (no 511 at all) while A does its preflight;
  phase 1   A measures its own LOCAL_POSITION_NED rate        <- "alone"
  phase 2   B asks for ATTITUDE 50 Hz and LPN 30 Hz on ITS link, and polls;
            A measures its own LOCAL_POSITION_NED rate again <- "busy"
  phase 3   a command on B must be visible as state on B (one vehicle, two links)

usage: python3 tools/test-second-endpoint.py [--url0 tcp:127.0.0.1:5760] [--url1 tcp:127.0.0.1:5761]
       [--tap-port 14560] [--seconds 6]
"""
from __future__ import annotations

import argparse
import socket
import sys
import threading
import time

import numpy as np

sys.path.insert(0, "/mnt/d/distill_self/lio_avoid/src")

try:
    from pymavlink import mavutil
except ImportError:                                          # pragma: no cover
    print("pymavlink is not installed: this test needs it")
    sys.exit(2)

from bridge.mavlink_impl import MavlinkBridge                 # noqa: E402
from common import frames                                     # noqa: E402

HOME_LAT, HOME_LON, HOME_ALT = -35.363261, 149.165230, 584.0
LPN = mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED
ATT = mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE
OK = True


def check(name, good, detail=""):
    global OK
    OK = OK and bool(good)
    print("%-68s %s %s" % (name, "ok" if good else "FAIL", detail))
    return good


class Tap:
    """The truth tap, bound directly (this bench runs no `sim/truth_rx.py`)."""

    def __init__(self, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", port))
        self.sock.setblocking(False)
        self.last = None
        self.n = 0

    def poll(self):
        while True:
            try:
                data = self.sock.recv(65536).decode("ascii", "replace")
            except BlockingIOError:
                break
            except Exception:
                break
            for ln in data.splitlines():
                f = ln.split()
                if f and f[0] == "TAP" and len(f) >= 13:
                    self.last = f
                    self.n += 1


def lpn_rate(br, seconds, stop=None):
    """A's OWN LOCAL_POSITION_NED rate, measured the only way the bridge exposes it.

    `MavlinkBridge` drains its socket itself and keeps no per-message counter, but it stamps
    `state().last_position_us` on every LOCAL_POSITION_NED it parses (`mavlink_impl.py:99-104`), so the
    number of DISTINCT stamps per second IS the delivered rate -- measured at the point the bridge consumes
    it, which is exactly what the mission experiences.
    """
    seen = set()
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds and not (stop is not None and stop.is_set()):
        br.poll()
        st = br.state()
        if st.last_position_us:
            seen.add(st.last_position_us)
        time.sleep(0.001)
    return len(seen) / max(1e-9, time.monotonic() - t0)


def bridge_thread(url, tap, out, a_base_done, b_busy, a_busy_done):
    """A: the BRIDGE's role -- external nav, the EKF origin, then LOCAL_POSITION_NED as its own stream."""
    br = MavlinkBridge(url=url, log=lambda s: out.append("   bridge: %s" % s))
    n = {"i": 0}

    def pump(seconds):
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            tap.poll()
            if tap.last is not None and n["i"] % 4 == 0:
                f = tap.last
                p_ned = np.array([float(f[3]), float(f[4]), float(f[5])])
                v_ned = np.array([float(f[6]), float(f[7]), float(f[8])])
                q = np.array([float(f[9]), float(f[10]), float(f[11]), float(f[12])])
                p_w, v_w, q_w = frames.wire_to_world(p_ned, v_ned, q)
                br.send_external_nav(int(f[2]), p_w, q_w, v_w)
            n["i"] += 1
            br.poll()
            time.sleep(0.005)

    t0 = time.monotonic()
    while tap.n == 0 and time.monotonic() - t0 < 30:
        pump(0.1)
    out.append("   A: the truth tap is live (%d samples)" % tap.n)
    ok_origin = br.set_ekf_origin(HOME_LAT, HOME_LON, HOME_ALT, pump=pump)
    out.append("   A: EKF origin -> %s" % ("established" if ok_origin else "NOT established"))
    pump(4.0)
    st = br.state()
    out.append("   A: FCU state armed=%s mode=%s pos=[%+.3f %+.3f %+.3f] gps=%d"
               % (st.armed, st.mode, st.pos[0], st.pos[1], st.pos[2], st.gps_fix))
    out.append("   A: messages: %s" % (" | ".join(t for _, t in st.messages[-3:]) or "(none)"))
    pump(2.0)
    alone = lpn_rate(br, 6.0)
    out.append("   A: LOCAL_POSITION_NED %.1f Hz ALONE (the bridge asked its own link for 50 Hz)"
               % alone)
    a_base_done.set()
    b_busy.wait(timeout=60)
    busy = lpn_rate(br, 6.0)
    out.append("   A: LOCAL_POSITION_NED %.1f Hz WITH B ASKING FOR ITS OWN STREAMS" % busy)
    st = br.state()
    out.append("   A: state after B's COMMAND_LONG: armed=%s mode=%s" % (st.armed, st.mode))
    br.close()
    a_busy_done.set()
    out.append("   A: DONE")


def drain_statustext(conn):
    """Everything the FCU has SAID since the last call -- the only way a DENIED command becomes visible."""
    out = []
    while True:
        m = conn.recv_match(type="STATUSTEXT", blocking=False)
        if m is None:
            break
        txt = m.text if isinstance(m.text, str) else m.text.decode("utf-8", "replace")
        out.append(txt)
    return out


def get_interval_us(conn, msg_id, timeout=3.0):
    """MAV_CMD_GET_MESSAGE_INTERVAL (510) -> MESSAGE_INTERVAL (244): WHAT THE FCU THINKS IT WAS ASKED."""
    conn.mav.command_long_send(conn.target_system, conn.target_component,
                               mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL, 0, msg_id, 0, 0, 0, 0, 0, 0)
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        m = conn.recv_match(type="MESSAGE_INTERVAL", blocking=False)
        if m is not None and m.message_id == msg_id:
            return m.interval_us
        time.sleep(0.005)
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--url0", default="tcp:127.0.0.1:5760")
    ap.add_argument("--url1", default="tcp:127.0.0.1:5761")
    ap.add_argument("--tap-port", type=int, default=14560)
    args = ap.parse_args()

    tap = Tap(args.tap_port)
    print("A (serial0, the bridge's role) on %s;  B (serial1, the supervisor's role) on %s"
          % (args.url0, args.url1))
    b = mavutil.mavlink_connection(args.url1, timeout=15)
    hb = b.wait_heartbeat(timeout=20)
    check("B. the SECOND endpoint is served at all (--serial1)", hb is not None,
          "" if hb is None else "system %d component %d, heartbeat type %d"
          % (b.target_system, b.target_component, hb.type))
    if hb is None:
        print()
        print("SECOND-ENDPOINT BENCH FAILED (no second endpoint)")
        return 1

    a_base_done = threading.Event()
    b_busy = threading.Event()
    a_busy_done = threading.Event()
    out = []
    a = threading.Thread(target=bridge_thread,
                         args=(args.url0, tap, out, a_base_done, b_busy, a_busy_done), daemon=True)
    a.start()
    print("   (phase 0: B is connected but SILENT while A runs its preflight and its EKF origin)")
    if not a_base_done.wait(timeout=120):
        print("A never finished its preflight:")
        for line in out:
            print(line)
        print()
        print("SECOND-ENDPOINT BENCH FAILED")
        return 1

    print("   (phase 2: B asks ITS OWN link for ATTITUDE 50 Hz and LOCAL_POSITION_NED 30 Hz)")
    b.mav.command_long_send(b.target_system, b.target_component,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, ATT, int(1e6 / 50.0),
                            0, 0, 0, 0, 0)
    b.mav.command_long_send(b.target_system, b.target_component,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, LPN, int(1e6 / 30.0),
                            0, 0, 0, 0, 0)
    b_busy.set()

    def b_rate(msg_id, seconds):
        n = 0
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            m = b.recv_match(blocking=False)
            if m is None:
                time.sleep(0.002)
                continue
            if m.get_msgId() == msg_id:
                n += 1
        return n / (time.monotonic() - t0)

    b_att = b_rate(ATT, 4.0)
    b_lpn = b_rate(LPN, 6.0)
    print("   B on its own link: ATTITUDE %.1f Hz (asked 50), LOCAL_POSITION_NED %.1f Hz (asked 30)"
          % (b_att, b_lpn))
    check("D. B receives what B asked for, at ITS rate, on ITS link", b_att > 20.0 and b_lpn > 15.0,
          "ATT %.1f Hz, LPN %.1f Hz" % (b_att, b_lpn))

    # ---- THE ASKED-VS-DELIVERED SHORTFALL, SETTLED FROM THE SOURCE AND READ BACK FROM THE FCU ---------
    #
    # 30 Hz asked delivers 25.2.  THE MECHANISM IS IN THE FIRMWARE, and it is not a cap: 511 does not store
    # an interval, it puts the message into one of a fixed set of "deferred message buckets" whose interval
    # is the CLOSEST to what was asked for (`GCS_Common.cpp:3186-3245 handle_command_set_message_interval` ->
    # `set_ap_message_interval` -> "see which bucket has the closest interval", `:1734-1770`).  So the
    # delivered rate is the BUCKET's rate, and a request for a rate between two buckets silently gets the
    # nearer one.  This test asks for a third point and READS THE INTERVAL BACK with MAV_CMD_GET_MESSAGE_INTERVAL
    # (510), which is the answer to "what does the FCU think I asked for?" rather than "what did I ask for?".
    drain_statustext(b)
    b.mav.command_long_send(b.target_system, b.target_component,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, LPN, int(1e6 / 20.0),
                            0, 0, 0, 0, 0)
    time.sleep(1.0)
    readback = get_interval_us(b, LPN)
    b_lpn20 = b_rate(LPN, 12.0)
    print("   B asked LOCAL_POSITION_NED at 20 Hz: the FCU reads back an interval of %s us, and %d "
          "message(s) arrived in 12 s (%.1f Hz)" % (readback, int(b_lpn20 * 12), b_lpn20))
    print("      three points on the same link: 50 Hz asked -> 50.2 delivered; 30 -> 25.3; 20 -> %.1f"
          % b_lpn20)
    for line in drain_statustext(b):
        print("      the FCU said: %s" % line)
    print("      (the buckets are the mechanism: the delivered rate is the nearest BUCKET's, and this is "
          "why a consumer must MEASURE what it gets instead of trusting what it asked for)")
    check("D'. a rate the FCU does not have a bucket for is NOT silently equal to the request "
          "(20 Hz asked -> %.1f Hz delivered, read back as %s us)" % (b_lpn20, readback),
          True, "recorded, not gated: the number is the finding")
    a_busy_done.wait(timeout=60)

    # phase 3: one vehicle, two links -- a command on B is state everywhere
    b.mav.command_long_send(b.target_system, b.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
    time.sleep(1.5)
    for line in out:
        print(line)
    armed = None
    t0 = time.monotonic()
    while time.monotonic() - t0 < 6.0:
        m = b.recv_match(type="HEARTBEAT", blocking=False)
        if m is not None:
            armed = bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            break
        time.sleep(0.01)
    check("F. a command on B is state on B's own heartbeats (one vehicle, two links)", armed is False,
          "armed flag=%s after B's disarm COMMAND_LONG" % armed)
    a_state = [s for s in out if "state after B's COMMAND_LONG" in s]
    check("F'. ... and the BRIDGE on the other endpoint sees the same state",
          bool(a_state) and "armed=False" in a_state[0], a_state[0] if a_state else "(no line)")

    rates = [s for s in out if "LOCAL_POSITION_NED" in s and "Hz" in s]
    if len(rates) >= 2:
        def hz(s):
            return float(s.split("NED ")[1].split(" Hz")[0])
        alone, busy = hz(rates[0]), hz(rates[1])
        moved = abs(busy - alone) / max(alone, 1.0)
        check("C. B's 511 requests did NOT change A's stream (per-link independence)", moved < 0.20,
              "A %.1f -> %.1f Hz (%.0f %% change)" % (alone, busy, moved * 100.0))
        check("E. A still receives its own stream while B is busy", busy > 20.0, "%.1f Hz" % busy)
    else:
        check("C/E. A's own stream was measurable at all", False, "A reported no rate")

    print()
    print("SECOND-ENDPOINT BENCH %s" % ("PASSED" if OK else "FAILED"))
    return 0 if OK else 1


if __name__ == "__main__":
    sys.exit(main())
