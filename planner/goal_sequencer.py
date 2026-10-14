#!/usr/bin/env python3
"""Feed ego-planner the mission's stops, one at a time -- and do nothing else.

The user's model of the division of labour (2026-10-14): *"ego只需要大概路径规划点吧，然后他会自己规划路径，
你不用搞自己避开圆柱啊"*.  So this node publishes stop i+1 once the vehicle has reached stop i, and that is
its whole job: no obstacle avoidance, no trajectory shaping, no goal surgery beyond the rule that is already
in the world file (a stop that lands inside a pillar is moved forward along the circle to that pillar's next
exit, `mission_b.circle.snap_need`, applied by `tools/world-circle.py`).

Interfaces (the frozen ones -- see PLAN.md 4.1 and ARCHITECTURE.md 4.0):
  out  /goal_pose   geometry_msgs/PoseStamped, frame world_w   -> planner/ego_adapter.py (which converts
                     to L with the SAME calibration file the FSM uses, and forwards it to the planner)
  in   /lio/robo/odom   nav_msgs/Odometry, frame lio_world     -> the LIO's own estimate, converted to W
                     with that same calibration, so "reached" is measured in the frame the stops are in

WHY IT REPEATS A GOAL: the adapter drops a goal that arrives before it has ever seen odometry (measured: an
unpatched planner segfaults on one), so the current stop is republished every `--repeat-s` seconds until it
is reached.  A dropped goal therefore costs at most one repeat interval.

WHY A TIMEOUT PER STOP: one unreachable stop must not end the mission.  A stop that is not reached within
`--timeout` seconds is logged with the distance that remained and the sequence moves on, so the flight is
scored on the stops it actually visited (that count is what the user asked for: "多少分之多少").

usage:
    planner/goal_sequencer.py --world worlds/forest_b.yaml --calib-file /mnt/d/distill_self/_stage/calib.json
    # add --dry-run to print the stop list and exit without ROS
"""
from __future__ import annotations

import argparse
import importlib.util
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)


def load_circle_module():
    """tools/world-circle.py is a script with a hyphen in its name, so import it by path."""
    path = os.path.join(ROOT, "tools", "world-circle.py")
    spec = importlib.util.spec_from_file_location("world_circle", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def load_calib(path):
    import json
    from common import frames
    with open(path) as fh:
        d = json.load(fh)
    return frames.Calib(R_WL=np.array(d["R_WL"], dtype=float),
                        t_WL=np.array(d["t_WL"], dtype=float),
                        n_samples=int(d.get("n_samples", 0)),
                        rms_m=float(d.get("rms_m", float("nan"))))


def l_to_w(calib, p_l):
    """L -> W, the same expression the adapter and the FSM use (`planner/ego_adapter.py:238`)."""
    return calib.R_WL @ np.asarray(p_l, dtype=float) + calib.t_WL


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default=os.path.join(ROOT, "worlds", "forest_b.yaml"))
    ap.add_argument("--calib-file", default="/mnt/d/distill_self/_stage/calib.json")
    ap.add_argument("--stops", default="",
                    help="explicit stop list 'E,N;E,N;...' overriding the world's mission_b.circle, with "
                         "--alt as their height.  WHY: a mission is not always a circle -- the user asked "
                         "for 'pad -> (-5,-5), hover 10 s, then (6,6)', and the stops then have to be "
                         "command-line data rather than a world-file geometry")
    ap.add_argument("--alt", type=float, default=2.0, help="height for --stops, m")
    ap.add_argument("--dwell-s", type=float, default=0.0,
                    help="seconds to HOLD at each stop after arriving, before commanding the next one "
                         "(the user's 'hover 10 s')")
    ap.add_argument("--stop-topic", default="/mission/stop",
                    help="publish the current stop index here as std_msgs/Int32 (0-based, -1 = the sequence "
                         "is complete).  The FSM follows trajectories until it reads -1, which is what keeps "
                         "its state and the mission's from drifting apart")
    ap.add_argument("--odom-topic", default="/lio/robo/odom")
    ap.add_argument("--goal-topic", default="/goal_pose",
                    help="where the stops are published (W); planner/ego_adapter.py subscribes to this and "
                         "converts to L with the same calibration the FSM uses")
    ap.add_argument("--arrive", type=float, default=0.35,
                    help="horizontal distance to the stop that counts as reached, m")
    ap.add_argument("--timeout", type=float, default=30.0,
                    help="seconds to wait for one stop before logging it as missed and moving on")
    ap.add_argument("--rate", type=float, default=10.0, help="check rate, Hz")
    ap.add_argument("--repeat-s", type=float, default=1.0, help="re-publish the current stop every this "
                                                                "many seconds (the adapter defers goals "
                                                                "that arrive before it has odometry)")
    ap.add_argument("--start-after", type=float, default=0.0,
                    help="seconds to wait before the first stop (the planner needs a warm-up)")
    ap.add_argument("--min-alt", type=float, default=-1.0,
                    help="do not send the FIRST stop until the vehicle is at least this high (m, U in W). "
                         "Default: the stops' own altitude minus 0.60 m.  MEASURED 2026-10-14: sending stop "
                         "1 while the vehicle was still on the pad put ego-planner into GEN_NEW_TRAJ and it "
                         "never produced a trajectory again -- 599 replans, 0 plan_success, the FSM waited "
                         "90 s for a trajectory that never came, and the flight had to be killed")
    ap.add_argument("--observe-wp-index", default="",
                    help="OBSERVER MODE: subscribe to ego's wp_index topic (patch 0008 publishes it) and "
                         "mirror it onto --stop-topic instead of publishing goals.  Use this once the "
                         "station list is passed to ego as fsm/waypoint* (tools/mission-params.py): the "
                         "sequence is then ego's, its advance test is a test on the vehicle, and this node "
                         "can no longer run ahead of the aircraft (MEASURED AO1: it walked stops 2..11 on "
                         "its own timeout while the vehicle hovered at stop 1).  Stops are still SCORED "
                         "here from truth -- reached, missed, and how long each took.")
    ap.add_argument("--log", default="", help="write the per-stop log here as well as stdout")
    ap.add_argument("--dry-run", action="store_true", help="print the stops and exit (no ROS)")
    args = ap.parse_args()

    wc = load_circle_module()
    if args.stops:
        stops = []
        for spec in args.stops.split(";"):
            if not spec.strip():
                continue
            v = [float(x) for x in spec.split(",")]
            stops.append([v[0], v[1], v[2] if len(v) > 2 else args.alt])
        print("goal_sequencer: %d stops from the command line: %s"
              % (len(stops), " -> ".join("(%.2f,%.2f,%.2f)" % tuple(s) for s in stops)), flush=True)
    else:
        stops = wc.commanded_points(args.world)
    if not stops:
        print("no mission_b.circle in %s and no --stops given" % args.world, file=sys.stderr)
        return 1
    if args.dry_run:
        for i, p in enumerate(stops):
            print("stop %2d  E %+8.4f  N %+8.4f  U %+6.3f" % (i, p[0], p[1], p[2]))
        return 0

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from std_msgs.msg import Int32

    class Sequencer(Node):
        def __init__(self):
            super().__init__("goal_sequencer")
            self.calib = load_calib(args.calib_file)
            # GOAL QUEUE DEPTH 10, NOT 1.  MEASURED 2026-10-14 (sortie Y1, the circle, with counters on every
            # link): this node published the stop-2 goal 42 times and the adapter's own counter shows it
            # RECEIVED 12 -- and the adapter forwarded all 12 (dedup 0) and ego saw 13, so nothing was lost
            # after the subscription.  RELIABLE with a history of 1 means the publisher DISCARDS an unacked
            # sample as soon as the next one is published, and the adapter is a single-threaded rclpy node
            # busy with 200 Hz odometry plus the 4 Hz clearance check, so 1 Hz goals were being overwritten
            # before they were ever delivered.  Depth 10 covers ~10 s of a 1 Hz goal and the goals are
            # identical, so a burst is harmless.
            # LATCHED GOAL PUBLISHER (TRANSIENT_LOCAL + RELIABLE).  MEASURED 2026-10-14 (sortie AJ1, the
            # circle): this node published stop 4's goal (W 0.000, 4.000) 42 times and the adapter received
            # NONE of them -- its own STATUS line reads "goals in 16, forwarded 16, dedup 0, deferred 0,
            # rejected 0", and the last goal it ever saw was stop 3 (W 1.5918, 3.6696).  The adapter is a
            # single-threaded rclpy node busy with 200 Hz odometry and its 4 Hz path/clearance work, so a
            # VOLATILE goal published while it is busy is gone for good; with TRANSIENT_LOCAL the middleware
            # keeps the latest sample for a subscriber that comes back, which is exactly the failure mode.
            # The consequence of the loss was severe and is why this matters: the adapter's own 1 Hz
            # re-forward (--reforward-hz) re-sent the LAST goal it had, i.e. stop 3, so ego's target stayed
            # on the stop it was already standing on and upstream's "close to goal" rule refused to plan
            # 54 737 times -- "PLANNER-STALL: NO trajectory for 52.0 s (target [3.67 -1.60 1.88],
            # odom [3.64 -1.63 1.75])", 0.14 m apart -- while the mission had moved on to stop 4.
            latched_goal = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                      reliability=ReliabilityPolicy.RELIABLE,
                                      durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pub = self.create_publisher(PoseStamped, args.goal_topic, latched_goal)
            # THE MISSION'S STATE, published so the FSM does not have to guess it.  MEASURED 2026-10-14:
            # with the FSM counting trajectories instead, four consecutive sorties aborted on a wait because
            # its counter had drifted out of step with the stops actually commanded.
            self.index_pub = self.create_publisher(Int32, args.stop_topic, 10)
            self.create_subscription(Odometry, args.odom_topic, self.on_odom, 10)
            self.ego_index = None
            if args.observe_wp_index:
                self.create_subscription(Int32, args.observe_wp_index, self.on_wp_index, 10)
                print("goal_sequencer: OBSERVER MODE: ego owns the sequence (%s); this node mirrors it onto "
                      "%s and scores the stops, it publishes NO goals"
                      % (args.observe_wp_index, args.stop_topic), flush=True)
            self.last_index = None
            self.here_w = None
            self.i = 0
            self.t_stop = None
            self.t_last_pub = 0.0
            self.t0 = time.monotonic()
            self.missed = []
            self.reached = []
            # THE FIRST STOP WAITS FOR ALTITUDE.  MEASURED 2026-10-14: with the first goal published while
            # the vehicle was still on the pad (0.16 m), ego-planner ended up in GEN_NEW_TRAJ and never
            # produced a trajectory again -- 599 replans, 0 plan_success, the FSM waited its full 90 s and
            # the flight had to be killed.  A planner that is asked to plan from the ground, inside its own
            # inflated floor, does not recover by itself.
            self.min_alt = (args.min_alt if args.min_alt >= 0.0
                            else float(max(s[2] for s in stops)) - 0.60)
            self.told_waiting = False
            self.done_logged = False
            self.t_reached = None                # the moment the current stop was first reached (dwell)
            self.fh = open(args.log, "w") if args.log else None
            self.create_timer(1.0 / max(args.rate, 1.0), self.tick)
            print("goal_sequencer: %d stops from %s, arrive %.2f m, timeout %.0f s, calib %s"
                  % (len(stops), os.path.basename(args.world), args.arrive, args.timeout,
                     args.calib_file), flush=True)

        def log(self, s):
            print("goal_sequencer: " + s, flush=True)
            if self.fh:
                self.fh.write("%.3f  %s\n" % (time.monotonic() - self.t0, s))
                self.fh.flush()

        def on_odom(self, msg):
            p_l = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y,
                            msg.pose.pose.position.z], dtype=float)
            self.here_w = l_to_w(self.calib, p_l)

        def on_wp_index(self, msg):
            """EGO owns the sequence (patch 0008): mirror its index onto --stop-topic, unchanged.

            The FSM's interface stays the same (`/mission/stop`, Int32, -1 = the tour is finished) -- but the
            number now comes from the planner that is actually flying the sequence, so it cannot run ahead.
            """
            self.ego_index = int(msg.data)
            m = Int32()
            m.data = self.ego_index
            self.index_pub.publish(m)
            if self.last_index != self.ego_index:
                self.last_index = self.ego_index
                self.log("ego wp_index %d (%s)" % (self.ego_index,
                                                   "tour complete" if self.ego_index < 0 else "running"))

        def pub_goal(self, p):
            if args.observe_wp_index:
                return                       # OBSERVER MODE: see --observe-wp-index
            m = PoseStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = "world_w"
            m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in p)
            m.pose.orientation.w = 1.0
            self.pub.publish(m)
            self.t_last_pub = time.monotonic()
            # COUNT THE PUBLICATIONS, because the goal path has three links whose numbers MUST agree and
            # MEASURED 2026-10-14 they did not: sortie V1 (the circle) reached ego's waypointCallback only
            # 8 times in 84 s while this node's repeat branch is supposed to publish the current stop every
            # --repeat-s (1.0 s) until it is reached, i.e. ~45 times for the 45 s stop-2 timeout.  Without
            # this counter the loss cannot be attributed to the sequencer, the transport, or the adapter.
            self.n_pub = getattr(self, "n_pub", 0) + 1

        def pub_index(self, i):
            """Publish the mission's stop index: 0-based while running, -1 when the sequence is complete.

            This is the FSM's only source of truth about where the mission is; it replaces the FSM's own
            trajectory counting, which drifted out of step and aborted four consecutive sorties.
            """
            if args.observe_wp_index:
                return                       # OBSERVER MODE: the index comes from ego (on_wp_index)
            if self.last_index != i:
                self.last_index = i
                m = Int32()
                m.data = int(i)
                self.index_pub.publish(m)

        def tick(self):
            now = time.monotonic()
            if now - self.t0 < args.start_after or self.here_w is None:
                return
            if self.i == 0 and self.t_stop is None and self.here_w[2] < self.min_alt:
                if not self.told_waiting:
                    self.told_waiting = True
                    self.log("holding the first stop until the vehicle is above %.2f m (now %.2f m): a "
                             "goal sent from the ground puts the planner into GEN_NEW_TRAJ for good"
                             % (self.min_alt, self.here_w[2]))
                return
            if self.i >= len(stops):
                # -1 on the stop topic is the FSM's signal that the mission is over and it should land.
                # The node keeps running (and keeps saying -1) instead of shutting itself down, because a
                # one-shot signal can be missed by a consumer that starts late or drops a message.
                self.pub_index(-1)
                if not self.done_logged:
                    self.done_logged = True
                    self.log("sequence complete: %d of %d stops reached, %d missed%s"
                             % (len(self.reached), len(stops), len(self.missed),
                                (" (%s)" % ", ".join("stop %d by %.2f m" % m for m in self.missed))
                                if self.missed else ""))
                return
            p = stops[self.i]
            self.pub_index(self.i)
            if self.t_stop is None:
                self.t_stop = now
                self.n_pub_at_stop = getattr(self, "n_pub", 0)   # count this stop's publications from here
                self.pub_goal(p)
                self.log("stop %d/%d  W[%+.3f %+.3f %+.3f]  (from %s)"
                         % (self.i + 1, len(stops), p[0], p[1], p[2],
                            "%.3f m away" % np.linalg.norm(self.here_w[:2] - np.array(p[:2]))
                            if self.here_w is not None else "no odom yet"))
            d = float(np.linalg.norm(self.here_w[:2] - np.array(p[:2])))
            # ALREADY REACHED: FINISH THE DWELL WHEREVER THE VEHICLE DRIFTS TO.  This is tested BEFORE the
            # arrival gate again, and that order is the whole fix.  MEASURED 2026-10-14 (sortie H-posvel, the
            # circle): the same stop was logged "REACHED in 4.4 s (0.237 m off) -- holding 5.0 s" and then,
            # 60 s later, "MISSED after 60 s: 0.606 m from it -- moving on" -- because the dwell was coded as
            # `if d <= arrive: ... return` and the FSM's station hold holds the PUBLISHED PLAN'S END, which
            # sits ~0.6 m PAST the stop.  The moment the vehicle drifted out of the 0.45 m gate the dwell
            # branch could never be entered again, so the arrival that had already happened was re-judged by
            # the timeout.  The mission then lost 60 s at the first stop (the next stop was published at
            # 74.7 s instead of ~28 s), and the FSM's own 60 s wait for a plan expired first:
            # "ABORT: no new trajectory on /planner/traj within 60 s after 2 trajectories (sequencer at
            # stop 1)" -- with the aircraft hovering 0.041 m from the stop for the whole minute.
            if self.t_reached is not None:
                if now - self.t_reached < args.dwell_s:
                    return                      # THE DWELL: stay on this stop, keep republishing it
                self.log("stop %d/%d dwell complete (%.3f m from it when the dwell ended)"
                         % (self.i + 1, len(stops), d))
                self.reached.append((self.i, now - self.t_stop))
                self.i += 1
                self.t_stop = None
                self.t_reached = None
                return
            if d <= args.arrive:
                self.t_reached = now
                if args.dwell_s > 0:
                    self.log("stop %d/%d REACHED in %.1f s (%.3f m off) -- holding %.1f s"
                             % (self.i + 1, len(stops), now - self.t_stop, d, args.dwell_s))
                    return
                self.log("stop %d/%d REACHED in %.1f s (%.3f m off)"
                         % (self.i + 1, len(stops), now - self.t_stop, d))
                self.reached.append((self.i, now - self.t_stop))
                self.i += 1
                self.t_stop = None
                self.t_reached = None
                return
            if now - self.t_stop > args.timeout:
                self.missed.append((self.i + 1, d))
                self.log("stop %d/%d MISSED after %.0f s: %.3f m from it -- moving on "
                         "(goal published %d time(s) for it)"
                         % (self.i + 1, len(stops), args.timeout, d,
                            getattr(self, "n_pub", 0) - getattr(self, "n_pub_at_stop", 0)))
                self.i += 1
                self.t_stop = None
                return
            if now - self.t_last_pub >= args.repeat_s:
                self.pub_goal(p)

    rclpy.init()
    node = Sequencer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:                       # the node shuts itself down when the sequence ends and
        # rclpy.spin then raises ExternalShutdownException; MEASURED 2026-10-14 that printed a traceback
        # over an otherwise clean "sequence complete" line, which reads like a crash and is not one.
        if "Shutdown" not in type(exc).__name__:
            raise
    if node.fh:
        node.fh.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
