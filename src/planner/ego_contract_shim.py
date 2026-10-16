#!/usr/bin/env python3
"""THE CONTRACT SHELL FOR ROUTE B: ego's OWN traj_server samples the plan; this only says WHO/WHERE/WHEN.

WHY THIS EXISTS AT ALL.  `traj_server` (ego's own, unmodified) turns `planning/bspline` into a 100 Hz
`quadrotor_msgs/PositionCommand` on `/position_cmd`, carrying **position AND velocity AND acceleration** -- the
reference ego actually wrote.  What it does NOT carry is our MISSION semantics, and it never will: it hard-codes
`trajectory_flag = READY` (`traj_server.cpp:211`) and forwards ego's per-replan `traj_id`, which changes about
once a second.  Two consumers need the other behaviour:

  * the SUPERVISOR ends the sortie on `trajectory_flag = COMPLETED` ("the plan box has declared the tour
    complete") and lands;
  * the acceptance report counts `trajectory_id` per WAYPOINT (`docs/PLANNERS.md`), which is what makes "which
    leg is this" answerable in a recording.

SO THIS SHELL DOES EXACTLY TWO THINGS, and nothing else:
  1. it OWNS THE TOUR -- sends one goal at a time on `/move_base_simple/goal` (in L, re-sent every 1 s until the
     aircraft arrives within `--arrive-m`, then a `--hold-s` dwell) and counts the waypoint index;
  2. it REWRITES THE CONTRACT on the way through -- copies `/position_cmd` to `/plan/cmd` verbatim, except
     `trajectory_id = waypoint index + 1` and `trajectory_flag = READY` while flying, `COMPLETED` once the last
     waypoint is reached and settled -- and it KEEPS TALKING after COMPLETED, because that rule is what let the
     supervisor tell "done" from "dead" in the first place.  **From COMPLETED on it also HOLDS**: the forwarded
     point is pinned to the last waypoint with zero velocity, zero acceleration and a latched yaw.  The contract
     asks the stream to keep TALKING, not to keep FLYING, and the sortie that showed the difference spent ~20 s of
     its landing window being thrown around by ego's own sweeping reference (round 81).

IT DOES NOT SAMPLE, FILTER, RATE-LIMIT OR PREDICT ANYTHING -- before COMPLETED.  If the numbers on the wire differ
from ego's own there, that is a bug in this file, not a design choice -- which is why every forwarded message is
also written to a CSV (`--trace-csv`), so the forwarding can be checked against `/position_cmd` rather than
trusted.  The post-COMPLETED hold is the ONE deliberate exception, it is a few lines, and the bench measures it:
after the flag turns COMPLETED the forwarded point is a single coordinate with |v| = 0 (`_stage/s12-rig.sh`).

THE POSE IT USES for arrival/dwell is the same state the planner is fed (`/ego/odom_fcu`, the FCU's own estimate
in L) -- the owner's call: the mission must not be sequenced on a different estimate from the one it plans on.

usage: ego_contract_shim.py --world worlds/diagonal.yaml --calib-file _stage/calib.json \
                            --odom-topic /ego/odom_fcu [--hold-s 5.0] [--arrive-m 0.30] [--trace-csv PATH]
       ego_contract_shim.py --selftest
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common import frames                                                       # noqa: E402

FRAME_L = "lio_world"
DEFAULT_CMD_TOPIC = "/plan/cmd"
DEFAULT_IN_TOPIC = "/position_cmd"          # traj_server's own publisher (traj_server.cpp:244)
DEFAULT_GOAL_TOPIC = "/move_base_simple/goal"
TRAJECTORY_STATUS_READY = 1                 # ego's own constants (traj_server.cpp:211)
TRAJECTORY_STATUS_COMPLETED = 3


class TourSequencer:
    """The mission's bookkeeping, with NO ROS IN IT, so `--selftest` can assert it with no graph and no flight.

    The rule for arrival is the adapter's, unchanged, because it is the rule the accepted sorties used: the
    vehicle must be within `arrive_m` AND slow (|v| < `settle_mps`), then it must STAY there for `hold_s` before
    the next waypoint is released.  `traj_server` has no opinion about any of this -- it flies whatever spline
    arrives, and when the spline runs out it hovers at its end (`traj_server.cpp:191-202`).
    """

    def __init__(self, targets_l, arrive_m=0.30, hold_s=5.0, settle_mps=0.08, goal_resend_s=1.0,
                 log=None, ego_driven=False):
        self.targets = np.asarray(targets_l, dtype=float).reshape(-1, 3)
        self.arrive_m = float(arrive_m)
        self.hold_s = float(hold_s)
        self.settle_mps = float(settle_mps)
        self.goal_resend_s = float(goal_resend_s)
        self.log = log or (lambda s: None)
        #: PLAN X: when True the TOUR IS EGO'S OWN -- `wp_index` drives the index and the end, this shell
        #: sends no goals and runs no arrival gate of its own.  Default False = the flown behaviour.
        self.ego_driven = bool(ego_driven)
        self.i_goal = 0
        self.t_dwell = None
        self.done = False
        self.p = None
        self.v = None
        self.t_last_goal = None
        self.goals_sent = 0
        self.n_ready = 0
        self.n_completed = 0
        #: how many `/wp_index` messages were seen (PLAN X's own health signal)
        self.wp_index_msgs = 0
        #: the yaw latched when the tour completed, so the post-COMPLETED hold cannot command a spin
        self._hold_yaw = None
        #: how many forwarded messages were rewritten to the hold (round 81)
        self.n_held_after = 0

    # ---- inputs ---------------------------------------------------------------------------------------
    def on_odom(self, p_l, v_l=None, now=0.0):
        self.p = np.asarray(p_l, dtype=float).reshape(3)
        self.v = None if v_l is None else np.asarray(v_l, dtype=float).reshape(3)
        self._sequence(now)

    def _sequence(self, now: float) -> None:
        if self.ego_driven:
            return                      # PLAN X: ego's `wp_index` is the only thing that advances this tour
        if self.done or self.p is None:
            return
        d = float(np.linalg.norm(self.p - self.target))
        speed = float(np.linalg.norm(self.v)) if self.v is not None else float("inf")
        if self.t_dwell is None:
            if d <= self.arrive_m and speed < self.settle_mps:
                self.t_dwell = now
                self.log("shim: ARRIVED at waypoint %d of %d (%.3f m), dwelling %.1f s"
                         % (self.i_goal + 1, len(self.targets), d, self.hold_s))
            return
        if now - self.t_dwell >= self.hold_s:
            if self.i_goal >= len(self.targets) - 1:
                self.done = True
                self.log("shim: TOUR COMPLETE -- trajectory_flag becomes COMPLETED and the stream KEEPS TALKING")
                return
            self.i_goal += 1
            self.t_dwell = None
            self.log("shim: next waypoint %d of %d" % (self.i_goal + 1, len(self.targets)))

    # ---- PLAN X: the tour driven by EGO's own sequencer -------------------------------------------------
    def on_wp_index(self, idx: int, now: float = 0.0) -> None:
        """`/wp_index` from PATCH 0011: the waypoint the mission is on, or -1 when the tour is finished.

        This is the whole of Plan X's shell: the id and the COMPLETED transition follow EGO, and nothing else
        in this file has an opinion about when the mission advances.  Note what is deliberately NOT done --
        the shell does not check an arrival of its own here, and does not require the vehicle to be anywhere:
        if ego says the tour is over, the contract says COMPLETED, and if ego's index is wrong that is a fact
        about ego which the trace should show rather than one this shell should hide.
        """
        self.wp_index_msgs += 1
        if int(idx) < 0:
            if not self.done:
                self.done = True
                self.i_goal = len(self.targets) - 1
                self.log("shim: TOUR COMPLETE reported BY EGO (wp_index -1 after %d wp_index message(s)) -- "
                         "trajectory_flag becomes COMPLETED and the stream KEEPS TALKING, held at waypoint %d"
                         % (self.wp_index_msgs, len(self.targets)))
            return
        new_i = max(0, min(int(idx), len(self.targets) - 1))
        if new_i != self.i_goal:
            self.log("shim: ego's wp_index advanced %d -> %d of %d: trajectory_id follows it"
                     % (self.i_goal + 1, new_i + 1, len(self.targets)))
            self.i_goal = new_i

    # ---- outputs --------------------------------------------------------------------------------------
    @property
    def target(self):
        return self.targets[min(self.i_goal, len(self.targets) - 1)]

    def goal_to_send(self, now: float):
        """The one goal ego should be working on, or None when the tour is over -- re-sent at `goal_resend_s`
        because ego's FSM falls back to WAIT_TARGET when it decides a plan has stalled, and only a NEW goal
        pulls it out (the same reason `planner/ego_source.py:480-493` re-sends).

        PLAN X: under `ego_driven` there is NOTHING to send -- ego's own sequencer owns the tour, and a goal
        sent to a FSM that is not subscribed to `/move_base_simple/goal` would be a message with no reader.
        The re-send cadence that used to drive the mission is exactly what Plan X removes."""
        if self.ego_driven or self.done:
            return None
        if self.t_last_goal is not None and (now - self.t_last_goal) < self.goal_resend_s:
            return None
        self.t_last_goal = now
        self.goals_sent += 1
        return self.target

    def rewrite(self, msg, now: float):
        """Apply the CONTRACT to one forwarded message: id per waypoint, READY -> COMPLETED.  In place.

        `trajectory_id` is `i_goal + 1`, so it advances exactly when the MISSION advances and at no other time
        (the adapter's rule, `ego_source.py:195`).  Everything else in the message is ego's own and is not
        touched -- including `header.frame_id`, which stays what `traj_server` wrote (`world`); the bridge
        accepts both names for this frame (`bridge/bridge_node.py:54-57`).

        ONE EXCEPTION, AND IT IS MEASURED (round 81): **after the tour is complete the message is rewritten to a
        HOLD.**  The contract requires the stream to keep TALKING after COMPLETED, not to keep FLYING, and the
        sortie that showed the difference (`DIAG34-routeb-pv`) spent **~20 s of the landing window being thrown
        around**: `setpoints.csv` records the commanded point going to (-2.5,-1.2), then (+0.4,-1.25), then
        (+1.1,-2.0) and back at 1.4-2.1 m/s, with the command's implied speed between messages reaching 4.25 m/s
        at the 95th percentile and 14.5 m/s at the maximum (median 0).  That was ego's own age rule sweeping and
        snapping (`traj_server.cpp:171-172`) -- motion the mission was over.  So from the moment `done` is set,
        the forwarded position is pinned to the LAST WAYPOINT (the point the aircraft has just arrived at, within
        the arrival gate), the velocity and acceleration are zeroed, and the yaw is latched at whatever the first
        post-COMPLETED message carried, so nothing spins during the landing.  Before COMPLETED the forwarding is
        still verbatim.
        """
        msg.trajectory_id = int(self.i_goal) + 1
        if self.done:
            if self._hold_yaw is None:
                self._hold_yaw = float(msg.yaw)          # latch once: a yaw jump mid-landing is a spin command
            p = self.targets[-1]
            msg.position.x, msg.position.y, msg.position.z = float(p[0]), float(p[1]), float(p[2])
            msg.velocity.x = msg.velocity.y = msg.velocity.z = 0.0
            msg.acceleration.x = msg.acceleration.y = msg.acceleration.z = 0.0
            msg.yaw, msg.yaw_dot = self._hold_yaw, 0.0
            msg.trajectory_flag = TRAJECTORY_STATUS_COMPLETED
            self.n_completed += 1
            self.n_held_after += 1
        else:
            msg.trajectory_flag = TRAJECTORY_STATUS_READY
            self.n_ready += 1
        return msg

    def summary(self):
        return ("shim: %d waypoint(s), %d goal(s) sent, %d READY and %d COMPLETED messages forwarded "
                "(%d of them held at the last waypoint), %s, tour source %s%s"
                % (len(self.targets), self.goals_sent, self.n_ready, self.n_completed, self.n_held_after,
                   "TOUR COMPLETE" if self.done else "waypoint %d" % (self.i_goal + 1),
                   "EGO (wp_index)" if self.ego_driven else "this shell",
                   " (%d wp_index message(s))" % self.wp_index_msgs if self.ego_driven else ""))


# -----------------------------------------------------------------------------------------------------------
def _targets_from_world(path, calib, z_default=2.0):
    w = yaml.safe_load(open(path))
    corners = w["mission"]["corners"]
    tw = np.array([[c[0], c[1], c[2] if len(c) > 2 else z_default] for c in corners], dtype=float)
    return (calib.R_WL.T @ (tw - calib.t_WL).T).T


def _selftest() -> int:
    bad = []
    t0 = 1000.0
    seq = TourSequencer([[0.0, 0.0, 2.0], [5.0, 0.0, 2.0]], arrive_m=0.30, hold_s=1.0, log=lambda s: print("   " + s))

    class Vec:                                      # the shape of PositionCommand's x/y/z triples
        def __init__(self):
            self.x = self.y = self.z = 0.0

    class M:                                        # a stand-in for PositionCommand, fields and all: the hold
        def __init__(self):                         # rewrite touches position/velocity/acceleration/yaw, so a
            self.trajectory_id = 0                  # two-field stub cannot exercise it (it raised AttributeError
            self.trajectory_flag = 0                # the first time this test ran, which is the test working)
            self.position, self.velocity, self.acceleration = Vec(), Vec(), Vec()
            self.yaw = 0.0
            self.yaw_dot = 0.0

    m = M()
    # THE FLIGHT'S OWN ORDER, because the first version of this test teleported the vehicle and asserted a
    # completion that a real sequence only reaches after the SECOND arrival:  arrive at WP1 -> dwell -> advance
    # -> arrive at WP2 -> dwell -> the tour is DONE.
    seq.on_odom([0.0, 0.0, 2.0], [0.0, 0.0, 0.0], now=t0)          # at WP1's target: arrival + dwell start
    seq.rewrite(m, now=t0)
    print("   at waypoint 1: id %d flag %d (READY=%d)" % (m.trajectory_id, m.trajectory_flag,
                                                          TRAJECTORY_STATUS_READY))
    if (m.trajectory_id, m.trajectory_flag) != (1, TRAJECTORY_STATUS_READY):
        bad.append("the first waypoint must be id 1 READY, got id %d flag %d" % (m.trajectory_id,
                                                                                m.trajectory_flag))
    seq.on_odom([5.0, 0.0, 2.0], [0.01, 0.0, 0.0], now=t0 + 0.5)   # mid-dwell, vehicle moved on: id must NOT move
    seq.rewrite(m, now=t0 + 0.5)
    if m.trajectory_id != 1:
        bad.append("the dwell changed the id to %d -- the consumer would think the mission advanced"
                   % m.trajectory_id)
    seq.on_odom([5.0, 0.0, 2.0], [0.01, 0.0, 0.0], now=t0 + 1.6)   # dwell over: the MISSION advances
    seq.rewrite(m, now=t0 + 1.6)
    print("   after waypoint 1's dwell: id %d flag %d (still READY)" % (m.trajectory_id, m.trajectory_flag))
    if (m.trajectory_id, m.trajectory_flag) != (2, TRAJECTORY_STATUS_READY):
        bad.append("advancing to waypoint 2 must give id 2 READY, got id %d flag %d" % (m.trajectory_id,
                                                                                       m.trajectory_flag))
    seq.on_odom([5.0, 0.0, 2.0], [0.01, 0.0, 0.0], now=t0 + 1.7)   # arrival at WP2 starts its dwell
    seq.on_odom([5.0, 0.0, 2.0], [0.01, 0.0, 0.0], now=t0 + 2.8)   # ... and the dwell ends the tour
    seq.rewrite(m, now=t0 + 2.8)
    print("   after the LAST waypoint's dwell: id %d flag %d (COMPLETED=%d)"
          % (m.trajectory_id, m.trajectory_flag, TRAJECTORY_STATUS_COMPLETED))
    if (m.trajectory_id, m.trajectory_flag) != (2, TRAJECTORY_STATUS_COMPLETED):
        bad.append("the last waypoint's dwell must complete the tour: got id %d flag %d"
                   % (m.trajectory_id, m.trajectory_flag))
    if seq.goal_to_send(now=t0 + 3.0) is not None:
        bad.append("a goal was offered after the tour was complete")
    # IT KEEPS TALKING: further messages are still forwarded, still COMPLETED -- AND IT NO LONGER MOVES (round 81)
    msg = m                                          # the same stand-in, reused with its new fields
    msg.position.x, msg.position.y, msg.position.z = 9.0, 9.0, 9.0
    msg.velocity.x, msg.velocity.y, msg.velocity.z = 3.0, 3.0, 3.0
    msg.acceleration.x, msg.acceleration.y = 5.0, 5.0
    msg.yaw = 1.234
    seq.rewrite(msg, now=t0 + 4.0)
    if msg.trajectory_flag != TRAJECTORY_STATUS_COMPLETED:
        bad.append("the stream stopped saying COMPLETED -- the supervisor needs it on every message")
    last_wp = seq.targets[-1]
    held = (float(msg.position.x), float(msg.position.y), float(msg.position.z))
    if max(abs(held[k] - float(last_wp[k])) for k in range(3)) > 1e-9:
        bad.append("after COMPLETED the forwarded point is not the last waypoint: %s vs %s"
                   % (held, tuple(float(x) for x in last_wp)))
    if any(abs(v) > 1e-12 for v in (msg.velocity.x, msg.velocity.y, msg.velocity.z)):
        bad.append("after COMPLETED the forwarded VELOCITY is not zero: %s"
                   % (msg.velocity.x, msg.velocity.y, msg.velocity.z))
    if any(abs(a) > 1e-12 for a in (msg.acceleration.x, msg.acceleration.y, msg.acceleration.z)):
        bad.append("after COMPLETED the forwarded ACCELERATION is not zero")
    yaw1 = float(msg.yaw)
    msg.position.x = -7.0
    msg.yaw = -3.0
    seq.rewrite(msg, now=t0 + 4.1)
    print("   after COMPLETED: point pinned to the last waypoint %s, |v| = 0, yaw latched at %.3f (second"
          " message keeps %.3f)"
          % (tuple(round(float(x), 3) for x in last_wp), yaw1, float(msg.yaw)))
    if abs(float(msg.yaw) - yaw1) > 1e-12:
        bad.append("the post-COMPLETED hold let the yaw move (%.3f -> %.3f)" % (yaw1, float(msg.yaw)))
    if seq.n_held_after < 2:
        bad.append("the hold counter did not move (%d)" % seq.n_held_after)
    # and the goal cadence
    seq2 = TourSequencer([[0.0, 0.0, 2.0], [5.0, 0.0, 2.0]], goal_resend_s=1.0, log=lambda s: None)
    seq2.on_odom([0.0, 0.0, 2.0], [0.0, 0.0, 0.0], now=t0)
    n_sent = sum(1 for k in range(30) if seq2.goal_to_send(now=t0 + 0.1 * k) is not None)
    print("   3 s of ticks at 10 Hz, goal_resend_s 1.0 -> %d goal(s) (expect 3-4)" % n_sent)
    if not 3 <= n_sent <= 4:
        bad.append("the goal cadence is %d in 3 s, expected 3-4" % n_sent)

    # ---- PLAN X: THE TOUR DRIVEN BY EGO'S OWN `wp_index` ------------------------------------------------
    # The assertions are the MODE'S CONTRACT, not a copy of the shell's: no goal may ever be sent, the id must
    # follow ego's index (including BACKWARDS, which the shell's own sequencer can never do and which would
    # otherwise be silently ignored), and -1 must complete the tour -- after which the hold pins the point.
    seq3 = TourSequencer([[0.0, 0.0, 2.0], [5.0, 0.0, 2.0], [9.0, 0.0, 2.0]],
                         log=lambda s: print("   " + s), ego_driven=True)
    if any(seq3.goal_to_send(now=t0 + 0.1 * k) is not None for k in range(30)):
        bad.append("PLAN X sent a goal -- ego's sequencer owns the tour, and /move_base_simple/goal has no "
                   "subscriber under flight_type 2")
    # the shell's own arrival gate must be DEAD in this mode: park the vehicle exactly on target 1 and tick
    # past any dwell -- the tour must NOT advance, because ego has not said so.
    for k in range(30):
        seq3.on_odom([0.0, 0.0, 2.0], [0.0, 0.0, 0.0], now=t0 + 0.5 * k)
    m3 = M()
    seq3.rewrite(m3, now=t0 + 15.0)
    if (m3.trajectory_id, seq3.done) != (1, False):
        bad.append("PLAN X advanced on the SHELL's arrival gate (id %d, done %s) -- the gate must be inert "
                   "when ego owns the tour" % (m3.trajectory_id, seq3.done))
    seq3.on_wp_index(1, now=t0 + 16.0)
    seq3.rewrite(m3, now=t0 + 16.1)
    if m3.trajectory_id != 2:
        bad.append("PLAN X did not follow ego's wp_index 1 -> id %d" % m3.trajectory_id)
    seq3.on_wp_index(0, now=t0 + 17.0)          # a BACKWARDS index: a wrong index must be SHOWN, not hidden
    seq3.rewrite(m3, now=t0 + 17.1)
    if m3.trajectory_id != 1:
        bad.append("PLAN X ignored a backwards wp_index (id %d, expected 1)" % m3.trajectory_id)
    seq3.on_wp_index(2, now=t0 + 18.0)
    seq3.on_wp_index(-1, now=t0 + 19.0)
    seq3.rewrite(m3, now=t0 + 19.1)
    hold = seq3.targets[-1]
    print("   PLAN X: ego drove the id to %d, then wp_index -1 -> flag %d, point pinned to %s, %d wp_index "
          "message(s)" % (m3.trajectory_id, m3.trajectory_flag,
                          tuple(round(float(x), 3) for x in hold), seq3.wp_index_msgs))
    if m3.trajectory_flag != TRAJECTORY_STATUS_COMPLETED:
        bad.append("PLAN X did not turn ego's wp_index -1 into COMPLETED (flag %d)" % m3.trajectory_flag)
    if max(abs(float(getattr(m3.position, c)) - float(hold[i])) for i, c in enumerate("xyz")) > 1e-9:
        bad.append("PLAN X's post-COMPLETED hold is not on the LAST waypoint")
    if seq3.n_held_after != 1:
        bad.append("PLAN X's hold counter is %d, expected 1" % seq3.n_held_after)
    if seq3.wp_index_msgs != 4:
        bad.append("PLAN X counted %d wp_index messages, expected 4" % seq3.wp_index_msgs)

    # ---- THE ARGUMENT SURFACE AND THE START-UP BANNERS ---------------------------------------------------
    # WHY THIS IS IN THE SELFTEST AT ALL: on 2026-10-16 the P3-2 banner referenced `args.goal_resend_s`, which
    # is not an argument, so the shell died at start-up with `AttributeError` in the FLOWN mode
    # (`--tour-source shell`) while this selftest passed -- it never built the banner.  A whole bench arm went
    # into finding that.  So: parse the real parser, and build BOTH banners from it.
    print("   -- the argument surface and the banners, built from the REAL parser")
    a = _parser().parse_args([])
    for field in ("world", "calib_file", "z", "hold_s", "arrive_m", "settle_mps", "in_topic", "cmd_topic",
                  "goal_topic", "odom_topic", "trace_csv", "tour_source", "wp_index_topic"):
        if not hasattr(a, field):
            bad.append("the start-up banner's argument surface has no %r -- main() would raise AttributeError "
                       "at start-up, in the flown mode, before the node spins" % field)
    if a.tour_source != "shell":
        bad.append("the default tour source is %r, not 'shell' (the flown behaviour must be the default)"
                   % a.tour_source)
    try:
        _print_p3_2_banner(a.arrive_m, a.settle_mps, a.hold_s)
        _print_tour_source_banner(a.tour_source, a.goal_topic, TourSequencer([[0, 0, 2]]).goal_resend_s,
                                  a.arrive_m, a.settle_mps, a.hold_s, a.wp_index_topic)
        _print_tour_source_banner("ego", a.goal_topic, 1.0, a.arrive_m, a.settle_mps, a.hold_s,
                                  a.wp_index_topic)
    except Exception as e:                                                       # noqa: BLE001
        bad.append("building the start-up banners raised %s: %s -- this is the exact failure that killed the "
                   "shell at start-up on 2026-10-16" % (type(e).__name__, e))
    if bad:
        print("\nCONTRACT SHIM SELFTEST FAILED (%d problem(s)):" % len(bad))
        for b in bad:
            print("   - %s" % b)
        return 1
    print("\nCONTRACT SHIM SELFTEST PASSED: id per waypoint, COMPLETED keeps talking WITHOUT moving, yaw latched, "
          "goal cadence")
    return 0


def _parser() -> "argparse.ArgumentParser":
    """THE ARGUMENT SURFACE, in one place, so `--selftest` can parse it and touch every field the start-up
    banner uses.  That is not tidiness: on 2026-10-16 the banner referenced `args.goal_resend_s`, which is NOT
    an argument (the sequencer's own constructor default is), so the shell died with `AttributeError` at
    start-up in `--tour-source shell` -- i.e. in the FLOWN configuration -- while `--selftest` passed, because
    the selftest never built the banner.  The next Plan X bench caught it only because it ran a control arm."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--world", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..",
                                                    "worlds", "diagonal.yaml"))
    ap.add_argument("--calib-file", default="/mnt/d/distill_self/_stage/calib.json")
    ap.add_argument("--z", type=float, default=2.0)
    ap.add_argument("--hold-s", type=float, default=5.0)
    ap.add_argument("--arrive-m", type=float, default=0.30)
    ap.add_argument("--settle-mps", type=float, default=0.08)
    ap.add_argument("--in-topic", default=DEFAULT_IN_TOPIC)
    ap.add_argument("--cmd-topic", default=DEFAULT_CMD_TOPIC)
    ap.add_argument("--goal-topic", default=DEFAULT_GOAL_TOPIC)
    ap.add_argument("--odom-topic", default="/ego/odom_fcu")
    ap.add_argument("--trace-csv", default="")
    ap.add_argument("--tour-source", choices=("shell", "ego"), default="shell",
                    help="WHO OWNS THE TOUR.  `shell` (default, the flown behaviour): this process sends the "
                         "goals and gates the arrivals.  `ego` (Plan X): ego's own waypoint sequencer owns it, "
                         "`--wp-index-topic` drives the id and the COMPLETED transition, and no goal is sent.")
    ap.add_argument("--wp-index-topic", default="/wp_index",
                    help="`std_msgs/Int32`, published by PATCH 0011: the current waypoint, -1 when finished")
    return ap


def _print_p3_2_banner(arrive_m: float, settle_mps: float, hold_s: float) -> None:
    """P3-2 OF THE MISSION-B REVIEW: "任务结束" is decided in THREE places whose thresholds differ, and two of
    them were only discoverable by reading three files.  They are printed HERE, once, at start-up, with the
    source of each -- so a run's own log says which rule ended it.  (The fourth number, the supervisor's
    SILENCE window, is a FAILURE rule, not an arrival rule, and is listed with the others so the difference is
    visible.)  It takes plain FLOATS rather than an `argparse.Namespace`, which is what makes it callable from
    `--selftest` -- see `_parser`."""
    print("shim: ARRIVAL RULES IN THIS STACK -- (1) EGO's own: `plan_done && wp_d_veh < "
          "fsm/waypoint_advance_m` (`ego_replan_fsm.cpp:879`, launch default 1.000 m, PATCH 0008's advance); "
          "(2) THIS SHELL's: d <= %.3f m AND |v| < %.3f m/s, held %.1f s (`--arrive-m/--settle-mps/--hold-s`); "
          "(3) the SUPERVISOR's: the contract's `trajectory_flag = COMPLETED` (`supervisor.py:295`) -- silence "
          "is NOT completion.  The supervisor's 3.0 s plan-silence window is a FAILURE rule, not an arrival "
          "rule." % (arrive_m, settle_mps, hold_s))


def _print_tour_source_banner(tour_source: str, goal_topic: str, goal_resend_s: float, arrive_m: float,
                              settle_mps: float, hold_s: float, wp_index_topic: str) -> None:
    """WHO OWNS THE TOUR, said out loud -- `shell` re-sends goals, `ego` (Plan X) sends none."""
    if tour_source == "ego":
        print("shim: TOUR SOURCE = EGO -- subscribed to %s (PATCH 0011); no goals are sent, and the id/flag "
              "follow ego's index.  Set --tour-source shell to restore the flown behaviour." % wp_index_topic)
    else:
        print("shim: TOUR SOURCE = THIS SHELL -- goals are re-sent on %s at %.1f Hz and the arrival gate is "
              "ours (%.3f m, |v| < %.3f m/s, held %.1f s)"
              % (goal_topic, 1.0 / goal_resend_s, arrive_m, settle_mps, hold_s))


def main() -> int:
    args = _parser().parse_args()
    if args.selftest:
        return _selftest()

    import json                                                                  # noqa: E402
    import rclpy                                                                 # noqa: E402
    from geometry_msgs.msg import PoseStamped                                    # noqa: E402
    from nav_msgs.msg import Odometry                                            # noqa: E402
    from quadrotor_msgs.msg import PositionCommand                               # noqa: E402
    from rclpy.node import Node                                                  # noqa: E402
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402

    d = json.load(open(args.calib_file))
    calib = frames.Calib(R_WL=np.array(d["R_WL"], dtype=float), t_WL=np.array(d["t_WL"], dtype=float))
    targets_l = _targets_from_world(args.world, calib, args.z)
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    except Exception:                                                            # noqa: BLE001
        rclpy.init(args=[])
    node = Node("ego_contract_shim")
    # LINE-BUFFERED STDOUT, because this process is normally ended by a signal and its banner is not flushed by
    # then: a bench arm on 2026-10-16 was diagnosed as "the shell printed nothing" when in fact it had printed
    # a full banner that was sitting in a 4 KB buffer.  An empty log must mean the process said nothing.
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:                                                            # noqa: BLE001
        pass
    ego_driven = (args.tour_source == "ego")
    seq = TourSequencer(targets_l, arrive_m=args.arrive_m, hold_s=args.hold_s,
                        settle_mps=args.settle_mps, log=lambda s: print(s, flush=True),
                        ego_driven=ego_driven)
    print("shim: %d waypoint(s) in L from %s" % (len(targets_l), args.world))
    for i, t in enumerate(targets_l):
        print("shim: waypoint %d -> %s (L)" % (i + 1, np.round(t, 3)))
    print("shim: TRAJ_SERVER's %s is forwarded to %s with trajectory_id per waypoint and a real COMPLETED"
          % (args.in_topic, args.cmd_topic))
    _print_p3_2_banner(args.arrive_m, args.settle_mps, args.hold_s)

    # the CONTRACT's QoS is not negotiable: RELIABLE + KEEP_LAST(5) + TRANSIENT_LOCAL, exactly as
    # `planner/ego_source.py:421-424` publishes it (a latched subscriber must still find the newest sample)
    cmd_qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(PositionCommand, args.cmd_topic, cmd_qos)
    goal_pub = node.create_publisher(PoseStamped, args.goal_topic, 10)
    # traj_server publishes with a plain depth-50 RELIABLE profile (`traj_server.cpp:244`)
    sub_qos = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE)
    trace = open(args.trace_csv, "w") if args.trace_csv else None
    if trace:
        trace.write("t_wall,wp,flag,px,py,pz,vx,vy,vz,ax,ay,az,yaw,jump_m,frame\n")
    last = [None]
    n = {"fwd": 0}

    def on_odom(m: Odometry):
        # A REPORTED ZERO IS "SLOW", NOT "UNKNOWN".  The first version passed `None` whenever the twist was
        # (0,0,0), and the sequencer reads `None` as "speed unknown" -- which BLOCKS the arrival and therefore the
        # whole tour.  In flight it worked only because the FCU's velocity noise is never exactly zero; a
        # stationary vehicle reporting a clean zero would have stalled the mission.  A message callback always
        # HAS a twist, so it is passed through as it is.
        v = [m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z]
        seq.on_odom([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z],
                    v, now=node.get_clock().now().nanoseconds * 1e-9)

    def on_cmd(m: PositionCommand):
        seq.rewrite(m, now=node.get_clock().now().nanoseconds * 1e-9)
        p = np.array([m.position.x, m.position.y, m.position.z])
        jump = 0.0 if last[0] is None else float(np.linalg.norm(p - last[0]))
        last[0] = p
        pub.publish(m)
        n["fwd"] += 1
        if trace:
            trace.write("%.4f,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s\n"
                        % (node.get_clock().now().nanoseconds * 1e-9, m.trajectory_id, m.trajectory_flag,
                           p[0], p[1], p[2], m.velocity.x, m.velocity.y, m.velocity.z,
                           m.acceleration.x, m.acceleration.y, m.acceleration.z, m.yaw, jump,
                           m.header.frame_id))

    node.create_subscription(Odometry, args.odom_topic, on_odom, sub_qos)
    node.create_subscription(PositionCommand, args.in_topic, on_cmd, sub_qos)
    if ego_driven:
        # PLAN X (PATCH 0011 publishes it): ego's own sequencer says which waypoint the mission is on, and -1
        # says it is over.  With this topic live the shell sends NO goals and runs NO arrival gate.
        from std_msgs.msg import Int32                                                # noqa: E402

        def on_wp_index(m: Int32):
            seq.on_wp_index(m.data, now=node.get_clock().now().nanoseconds * 1e-9)

        node.create_subscription(Int32, args.wp_index_topic, on_wp_index, sub_qos)
    _print_tour_source_banner(args.tour_source, args.goal_topic, seq.goal_resend_s, args.arrive_m,
                              args.settle_mps, args.hold_s, args.wp_index_topic)

    def tick():
        g = seq.goal_to_send(now=node.get_clock().now().nanoseconds * 1e-9)
        if g is None:
            return
        gm = PoseStamped()
        gm.header.stamp = node.get_clock().now().to_msg()
        gm.header.frame_id = FRAME_L
        gm.pose.position.x, gm.pose.position.y, gm.pose.position.z = float(g[0]), float(g[1]), float(g[2])
        gm.pose.orientation.w = 1.0
        goal_pub.publish(gm)
        # SAY IT.  A goal that is published and never seen by the FSM (wrong topic, no subscriber, the node
        # still in a constructor wait) looks EXACTLY like a planner that refuses to plan -- a whole bench arm
        # was spent on that ambiguity on 2026-10-16 (the control arm of `_stage/s13-planx.sh` never planned and
        # nothing in either log said whether a goal had been sent).  One line per send, at 1 Hz, is cheap.
        print("shim: goal %d/%d -> (%.3f, %.3f, %.3f) L on %s"
              % (seq.i_goal + 1, len(seq.targets), float(g[0]), float(g[1]), float(g[2]), args.goal_topic),
              flush=True)

    node.create_timer(0.1, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    print(seq.summary(), flush=True)
    if trace:
        trace.close()
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
