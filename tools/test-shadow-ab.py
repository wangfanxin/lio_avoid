#!/usr/bin/env python3
"""Does the shadow command stream change what the PLAN box publishes on `/planner/traj`?  Byte for byte?

THE CLAIM THIS TESTS (and it is a claim, so it is tested and not asserted).  Step S3 adds a SECOND output
to `planner/rectangle_source.py`: `quadrotor_msgs/msg/PositionCommand` on `/plan/cmd`, which nothing
subscribes to yet.  The invariant that makes that safe is stronger than "it should not matter": the Path
message sequence the FSM consumes must be **byte-for-byte what it was**, because that sequence IS the
flying plan and the flown configuration has been scored against it.  Two ways to be wrong are both
invisible in a flight until something diverges:

  * the PACING moved -- the leg/dwell state machine was extracted into
    `planner/cmd_stream.py::TourPacer`, and a leg could now arm or advance on a different tick;
  * the second publisher's work could perturb the loop, since the Path is republished from the same
    1/--rate loop that now also emits the stream.

HOW IT IS MEASURED.  The real producer is imported with **`rclpy` replaced by a recording stub** and
**`time` replaced by a clock this test owns**, so a whole 4-corner tour -- its dwells and its COMPLETED
statement included -- runs in milliseconds, deterministically, with no ROS graph, no DDS, no SITL and no
airframe.  Three runs over the SAME seeded flight:

    A  --cmd-stream                        what the refactor ships
    B  --no-cmd-stream                     the producer with the Path only (the pre-S3 output)
    C  --no-cmd-stream, arrival gate 0.02  the NEGATIVE CONTROL

and the recorded Path publications are compared as exact strings.  C exists because "A == B" is also what
a test that looked at nothing would report: C must DIFFER, or the comparison is not reading the plan.

WHAT IS RECORDED, per publication: the producer's own clock, the message's frame_id and stamp, and the
exact position and orientation of every pose in order.  That is the whole of what a consumer of
`/planner/traj` can see -- `fsm/mission_fsm.py` reads `header.frame_id`, `header.stamp` and the poses, and
`tools/record-traj.py` reads the poses.

usage: tools/test-shadow-ab.py
"""
from __future__ import annotations

import argparse
import contextlib
import importlib.util
import io
import json
import os
import sys
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)


# ---------------------------------------------------------------------------------------------
# the clock: the producer's only source of time
# ---------------------------------------------------------------------------------------------
class FakeClock:
    def __init__(self):
        self.t = 100.0

    def monotonic(self):
        return self.t

    def sleep(self, s):
        self.t += float(s)


class FakeTime:
    """Stands in for the `time` module INSIDE the producer: its `import time` finds this in
    `sys.modules` (the import is inside `main()`, so the substitution lands before it runs), and every
    `time.monotonic()` / `time.sleep()` in the node is therefore ours."""

    def __init__(self, clock):
        self._c = clock

    def monotonic(self):
        return self._c.monotonic()

    def sleep(self, s):
        self._c.sleep(s)

    def time(self):
        return self._c.t


# ---------------------------------------------------------------------------------------------
# the ROS stub: just enough of rclpy and of the three message types, plus a recorder for publish()
# ---------------------------------------------------------------------------------------------
class _Vec3:
    def __init__(self):
        self.x = self.y = self.z = 0.0


class _Quat:
    def __init__(self):
        self.x = self.y = self.z = 0.0
        self.w = 1.0


class _Pose:
    def __init__(self):
        self.position = _Vec3()
        self.orientation = _Quat()


class _Stamp:
    def __init__(self):
        self.sec = 0
        self.nanosec = 0


class _Header:
    def __init__(self):
        self.stamp = _Stamp()
        self.frame_id = ""


class _PoseStamped:
    def __init__(self):
        self.header = _Header()
        self.pose = _Pose()


class FakePath:
    def __init__(self):
        self.header = _Header()
        self.poses = []


class FakeCmd:
    def __init__(self):
        self.header = _Header()
        self.position = _Vec3()
        self.velocity = _Vec3()
        self.acceleration = _Vec3()
        self.yaw = 0.0
        self.yaw_dot = 0.0
        self.kx = [0.0, 0.0, 0.0]
        self.kv = [0.0, 0.0, 0.0]
        self.trajectory_id = 0
        self.trajectory_flag = 0


class _Linear:
    def __init__(self):
        self.x = self.y = self.z = 0.0


class _Twist:
    def __init__(self):
        self.linear = _Linear()


class _PoseWithCov:
    """The inner `pose` of an Odometry: `m.pose.pose.position` is what the producer reads."""

    def __init__(self):
        self.pose = _Pose()


class FakeOdom:
    def __init__(self):
        self.pose = _PoseWithCov()
        self.twist = _Twist()


class _RosTime:
    """rclpy's `Time`: what `node.get_clock().now()` returns.  `.to_msg()` is how every publisher here
    reads it, so the stub provides exactly that and nothing else."""

    def __init__(self, t):
        self._t = t

    def to_msg(self):
        m = _Stamp()
        m.sec = int(self._t)
        m.nanosec = int((self._t - int(self._t)) * 1e9)
        return m


class _Clock:
    def __init__(self, clock):
        self._c = clock

    def now(self):
        return _RosTime(self._c.t)


class _Record:
    def __init__(self, clock, kind=""):
        self.clock = clock
        self.kind = kind
        self.pub = []

    def publish(self, msg):
        self.pub.append((self.clock.t, snapshot(msg)))


class FakeNode:
    def __init__(self, name, clock, topic, cmd_topic):
        self.name = name
        self.clock = clock
        self.record = _Record(clock, "path")
        self.cmd_record = _Record(clock, "cmd")
        self.subs = {}
        self.pubs = []

    def create_publisher(self, typ, topic, qos):
        if typ is FakePath:
            self.pubs.append(self.record)
            return self.record
        return self.cmd_record

    def create_subscription(self, typ, topic, cb, qos):
        self.subs[topic] = cb
        return cb

    def get_clock(self):
        return _Clock(self.clock)


def snapshot(msg):
    """EVERYTHING a consumer of these messages can see, as one canonical string."""
    if isinstance(msg, FakePath):
        return json.dumps({
            "type": "Path",
            "frame_id": msg.header.frame_id,
            "stamp": [msg.header.stamp.sec, msg.header.stamp.nanosec],
            "poses": [{
                "frame_id": ps.header.frame_id,
                "stamp": [ps.header.stamp.sec, ps.header.stamp.nanosec],
                "p": [repr(float(ps.pose.position.x)), repr(float(ps.pose.position.y)),
                      repr(float(ps.pose.position.z))],
                "q": [repr(float(ps.pose.orientation.w)), repr(float(ps.pose.orientation.x)),
                      repr(float(ps.pose.orientation.y)), repr(float(ps.pose.orientation.z))],
            } for ps in msg.poses],
        }, sort_keys=True)
    return json.dumps({
        "type": "PositionCommand",
        "frame_id": msg.header.frame_id,
        "stamp": [msg.header.stamp.sec, msg.header.stamp.nanosec],
        "position": [repr(float(msg.position.x)), repr(float(msg.position.y)),
                     repr(float(msg.position.z))],
        "velocity": [repr(float(msg.velocity.x)), repr(float(msg.velocity.y)),
                     repr(float(msg.velocity.z))],
        "acceleration": [repr(float(msg.acceleration.x)), repr(float(msg.acceleration.y)),
                         repr(float(msg.acceleration.z))],
        "yaw": repr(float(msg.yaw)),
        "yaw_dot": repr(float(msg.yaw_dot)),
        "kx": [repr(float(v)) for v in msg.kx],
        "kv": [repr(float(v)) for v in msg.kv],
        "trajectory_id": int(msg.trajectory_id),
        "trajectory_flag": int(msg.trajectory_flag),
    }, sort_keys=True)


# ---------------------------------------------------------------------------------------------
# the driver: import the REAL producer and run it on the fake clock
# ---------------------------------------------------------------------------------------------
def fake_modules(clock, holder):
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda *a, **k: None

    def ok():
        holder["n"] = holder.get("n", 0) + 1
        return holder["n"] <= holder["max_iters"]

    def spin_once(node, timeout_sec=0.0):
        """The producer's own tick.  The seeded vehicle moves here, so it advances at the LOOP's rate
        (1/--rate), which is the rate the real odometry would arrive at relative to this loop."""
        cb = holder.get("odom_cb")
        if cb is not None:
            holder["step"](cb)

    rclpy.ok = ok
    rclpy.spin_once = spin_once

    node_mod = types.ModuleType("rclpy.node")

    class Node:
        def __new__(cls, name):
            holder["node"] = FakeNode(name, clock, holder["topic"], holder["cmd_topic"])
            return holder["node"]

    node_mod.Node = Node

    # `planner.cmd_stream` is passed through unchanged EXCEPT that the pacer the producer builds is
    # captured, so the seeded vehicle can follow the producer's own leg index instead of guessing the
    # tour (see `step` below).
    import planner.cmd_stream as real_cmd_stream
    cmd_mod = types.ModuleType("planner.cmd_stream")
    cmd_mod.__dict__.update(real_cmd_stream.__dict__)

    class CapturingPacer(real_cmd_stream.TourPacer):
        def __init__(self, *a, **kw):
            super().__init__(*a, **kw)
            holder["pacer"] = self
            st = holder.get("state")
            if st is not None:
                st["pacer"] = self

    cmd_mod.TourPacer = CapturingPacer
    planner_pkg = types.ModuleType("planner")
    planner_pkg.__path__ = [os.path.join(ROOT, "planner")]

    qos_mod = types.ModuleType("rclpy.qos")

    class QoSProfile:
        def __init__(self, **kw):
            self.kw = kw

    def _enum(*names):
        return type("E", (), {n: n for n in names})

    qos_mod.QoSProfile = QoSProfile
    qos_mod.ReliabilityPolicy = _enum("RELIABLE", "BEST_EFFORT")
    qos_mod.HistoryPolicy = _enum("KEEP_LAST", "KEEP_ALL")
    qos_mod.DurabilityPolicy = _enum("TRANSIENT_LOCAL", "VOLATILE")

    sig_mod = types.ModuleType("rclpy.signals")
    sig_mod.SignalHandlerOptions = _enum("NO")

    nav = types.ModuleType("nav_msgs.msg")
    nav.Path, nav.Odometry = FakePath, FakeOdom
    geo = types.ModuleType("geometry_msgs.msg")
    geo.PoseStamped = _PoseStamped
    quad = types.ModuleType("quadrotor_msgs.msg")
    quad.PositionCommand = FakeCmd

    return {
        "rclpy": rclpy, "rclpy.node": node_mod, "rclpy.qos": qos_mod,
        "rclpy.signals": sig_mod, "nav_msgs": types.ModuleType("nav_msgs"),
        "nav_msgs.msg": nav, "geometry_msgs": types.ModuleType("geometry_msgs"),
        "geometry_msgs.msg": geo, "quadrotor_msgs": types.ModuleType("quadrotor_msgs"),
        "quadrotor_msgs.msg": quad, "time": FakeTime(clock),
        "planner.cmd_stream": cmd_mod, "planner": planner_pkg,
    }


def run_once(world, calib_file, cmd_stream, arrive_m, rate, iters, loop_rate=4.0):
    """One run of the real producer.  Returns (path_pubs, cmd_pubs, log_lines, state, out)."""
    from common import frames
    d = json.load(open(calib_file))
    calib = frames.Calib(R_WL=np.array(d["R_WL"]), t_WL=np.array(d["t_WL"]))

    clock = FakeClock()
    holder = {"topic": "/planner/traj", "cmd_topic": "/plan/cmd", "odom_topic": "/lio/robo/odom",
              "max_iters": iters, "n": 0}

    # load the producer with the stubs in place
    fakes = fake_modules(clock, holder)
    saved = {k: sys.modules.get(k) for k in list(fakes) + ["bridge", "bridge.base"]}
    sys.modules.update(fakes)
    for k in ("rectangle_source_ab", "planner.rectangle_source"):
        sys.modules.pop(k, None)
    spec = importlib.util.spec_from_file_location(
        "rectangle_source_ab", os.path.join(ROOT, "planner", "rectangle_source.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    corners, _ = mod.load_square(world)
    corners = np.asarray(corners, dtype=float)

    # THE SEEDED FLIGHT is a CLOSED LOOP around the producer's own pacer: the vehicle always drives
    # toward the corner the producer is currently asking for, at 0.30 m/s, and stands still while the
    # pacer dwells.  Reading the target from `pacer.i_leg` is what makes the run a flight rather than a
    # script -- when the producer advances a leg, the vehicle is already standing on that corner's start
    # (it arrived there), and when the producer dwells, the vehicle waits.  MEASURED: a scripted version
    # that advanced the vehicle on its own leg index stalled at leg 0, because it and the producer were
    # counting different tours.
    st = {"last_t": clock.t, "p": corners[0].copy(), "advances": 0, "seen_leg": 0,
          "pacer": None}
    holder["state"] = st

    def step(cb):
        dt = clock.t - st["last_t"]
        st["last_t"] = clock.t
        pacer = holder.get("pacer")
        if pacer is not None and pacer.i_leg != st["seen_leg"]:
            # the producer has advanced: the vehicle is standing on the new leg's start corner
            st["seen_leg"] = pacer.i_leg
            st["p"] = corners[pacer.i_leg % len(corners)].copy()
            st["advances"] += 1
        if pacer is not None and not pacer.done and not pacer.dwelling:
            target = corners[(pacer.i_leg + 1) % len(corners)]
            dv = target - st["p"]
            dist = float(np.linalg.norm(dv))
            if dist <= 0.30 * dt or dist < 1e-12:
                st["p"] = target.copy()
            else:
                st["p"] = st["p"] + dv / dist * (0.30 * dt)
        # what the LIO publishes is that point in L; the producer converts it back to W itself
        p_l = calib.R_WL.T @ (st["p"] - calib.t_WL)
        m = FakeOdom()
        m.pose.pose.position.x = float(p_l[0])
        m.pose.pose.position.y = float(p_l[1])
        m.pose.pose.position.z = float(p_l[2])
        cb(m)

    holder["step"] = step
    argv = ["rectangle_source.py", "--world", world, "--calib-file", calib_file,
            "--hold-s", "5.0", "--arrive-m", repr(arrive_m), "--rate", repr(loop_rate),
            "--cmd-rate", repr(rate), "--odom", "/lio/robo/odom"]
    if not cmd_stream:
        argv.append("--no-cmd-stream")
    # THE SEEDED VEHICLE MUST BE REGISTERED BEFORE `main()` RUNS: the producer subscribes inside
    # `main()` and then spins in its loop, so the callback has to reach the stub before the call.
    # MEASURED: wiring it afterwards leaves the producer with no pose at all, the first leg never arms,
    # and the run publishes one Path message and nothing else.
    _saved_create = FakeNode.create_subscription

    def create_subscription(self, typ, topic, cb, qos):
        self.subs[topic] = cb
        if topic == holder["odom_topic"]:
            holder["odom_cb"] = cb
        return cb

    FakeNode.create_subscription = create_subscription
    try:
        old_argv = sys.argv
        sys.argv = argv
        log = io.StringIO()
        try:
            with contextlib.redirect_stdout(log):
                try:
                    mod.main()
                except SystemExit:
                    pass
        finally:
            sys.argv = old_argv
    finally:
        FakeNode.create_subscription = _saved_create
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
    node = holder["node"]
    n_loop = holder.get("n", 0)
    p = holder.get("pacer")
    out = {"n_loop": n_loop,
           "path_until": node.record.pub[-1][0] if node.record.pub else None,
           "cmd_until": node.cmd_record.pub[-1][0] if node.cmd_record.pub else None,
           "pacer": (p.i_leg, p.armed, p.dwelling, p.done, p.t_done) if p is not None else None,
           "advances": st["advances"]}
    return node.record.pub, node.cmd_record.pub, log.getvalue().splitlines(), st, out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default=os.path.join(ROOT, "worlds", "pillars.yaml"))
    ap.add_argument("--calib-file", default="/mnt/d/distill_self/_stage/calib.json")
    ap.add_argument("--rate", type=float, default=100.0)
    ap.add_argument("--sim-seconds", type=float, default=60.0,
                    help="how much SIMULATED time each run covers.  The loop's period is 1/--rate, so "
                         "the iteration count is derived from this: a run at a 100 Hz loop needs 100 "
                         "iterations per simulated second and would otherwise cover 1/25th of the tour")
    ap.add_argument("--iters", type=int, default=0,
                    help="override the iteration count (0 = derive it from --sim-seconds and the rate)")
    args = ap.parse_args()
    ok = True

    print("THE SHADOW CLAIM: /plan/cmd runs BESIDE /planner/traj, and the Path does not move.")
    print("world %s" % args.world)
    print("calib %s" % args.calib_file)
    print()
    # THE LOOP RUNS AT `--rate` (4 Hz is the flown republish rate) AND THE STREAM DECLARES `--cmd-rate`
    # (100 Hz, ego's timer).  BOTH variants are run, because they answer different questions: at a 4 Hz
    # loop the stream can only emit one sample per iteration, so its measured rate is the LOOP's -- and
    # the same run at a 100 Hz loop must emit ~2x as many samples for the SAME Path, which is what shows
    # the stream's clock is independent of the Path's.
    runs = {}
    plan = (("A  --cmd-stream, loop 4 Hz, cmd 100 Hz", True, 0.10, 4.0),
            ("B  --no-cmd-stream, loop 4 Hz", False, 0.10, 4.0),
            ("C  --no-cmd-stream, arrive-m 0.02 (NEGATIVE CONTROL)", False, 0.02, 4.0),
            ("D  --cmd-stream, loop 100 Hz, cmd 100 Hz", True, 0.10, 100.0))
    labels = {}
    for label, cmd, arrive, loop_rate in plan:
        iters = args.iters or int(args.sim_seconds * loop_rate) + 1
        paths, cmds, logs, st, out = run_once(args.world, args.calib_file, cmd, arrive, args.rate,
                                             iters, loop_rate=loop_rate)
        runs[label] = (paths, cmds, logs)
        labels[label] = (loop_rate, arrive, cmd)
        print("%-52s loop %4d  Path %4d  cmd %5d  logs %3d  legs+%d  pacer=%s"
              % (label, out["n_loop"], len(paths), len(cmds), len(logs), out["advances"],
                 out["pacer"]))
        if not out["pacer"] or not out["pacer"][3]:
            print("   FAIL the tour did not complete in this run -- the harness is not flying the "
                  "mission it compares")
            ok = False

    a_paths, a_cmds, a_logs = runs["A  --cmd-stream, loop 4 Hz, cmd 100 Hz"]
    b_paths, b_cmds, b_logs = runs["B  --no-cmd-stream, loop 4 Hz"]
    c_paths, c_cmds, c_logs = runs["C  --no-cmd-stream, arrive-m 0.02 (NEGATIVE CONTROL)"]
    d_paths, d_cmds, d_logs = runs["D  --cmd-stream, loop 100 Hz, cmd 100 Hz"]

    print()
    print("--- 1. the Path sequences, compared byte for byte (frame, stamps, every pose) ---")
    if not a_paths or not b_paths:
        print("   FAIL no Path publications were recorded (A=%d, B=%d) -- the harness is broken"
              % (len(a_paths), len(b_paths)))
        ok = False
    elif len(a_paths) != len(b_paths):
        print("   FAIL A published %d Path messages, B published %d" % (len(a_paths), len(b_paths)))
        ok = False
    else:
        diffs = [i for i, (x, y) in enumerate(zip(a_paths, b_paths)) if x != y]
        print("   %d Path publications compared; %d differ" % (len(a_paths), len(diffs)))
        for i in diffs[:3]:
            print("      #%d A=%s" % (i, a_paths[i][1][:200]))
            print("      #%d B=%s" % (i, b_paths[i][1][:200]))
        if diffs:
            print("   FAIL the Path moved when the shadow stream was enabled")
            ok = False
        else:
            print("   IDENTICAL: enabling /plan/cmd does not change one byte of the flown plan")

    print()
    print("--- 2. the NEGATIVE CONTROL: the comparison must be able to SEE a change ---")
    same = (len(c_paths) == len(a_paths) and all(x == y for x, y in zip(a_paths, c_paths)))
    if same:
        print("   FAIL the arrive-m control produced the SAME Path sequence -- the comparison is blind")
        ok = False
    else:
        print("   DIFFERENT as required: %d vs %d Path publications; a tighter arrival gate changes the"
              " plan" % (len(a_paths), len(c_paths)))
        print("   (so the identity in 1 is a measurement, not a tautology)")

    print()
    print("--- 3. the shadow's own footprint on the log ---")
    extra = [l for l in a_logs if l not in b_logs]
    missing = [l for l in b_logs if l not in a_logs]
    print("   lines present only WITH the stream: %d" % len(extra))
    for l in extra:
        print("      %s" % l[:150])
    print("   lines present only WITHOUT the stream: %d" % len(missing))
    for l in missing:
        print("      %s" % l[:150])
    if missing:
        print("   FAIL the stream REMOVED log lines")
        ok = False
    if len(extra) != 1:
        print("   FAIL expected exactly ONE added line (the stream's own announcement), got %d"
              % len(extra))
        ok = False
    if len(a_cmds) == 0:
        print("   FAIL the --cmd-stream half published no command samples")
        ok = False
    if len(b_cmds) != 0 or len(c_cmds) != 0:
        print("   FAIL a --no-cmd-stream half published command samples (B=%d, C=%d)"
              % (len(b_cmds), len(c_cmds)))
        ok = False
    print("   command samples: A (loop 4 Hz) %d, B %d, C %d, D (loop 100 Hz) %d"
          % (len(a_cmds), len(b_cmds), len(c_cmds), len(d_cmds)))

    print()
    print("--- 4. the stream's clock is its OWN, not the Path's ---")
    # At a 4 Hz loop the stream can only emit once per iteration, so "241 samples for 171 Path messages"
    # is the LOOP's rate and not a defect: the stream is due on every pass.  Running the same tour at a
    # 100 Hz loop emits ~25x as many samples, which is what separates "self-clocked at 100 Hz" from "the
    # loop happens to be slow".
    if len(d_cmds) <= 5 * len(a_cmds):
        print("   FAIL a 100 Hz loop emitted %d samples against %d at 4 Hz: the stream is not clocked by"
              " its own rate" % (len(d_cmds), len(a_cmds)))
        ok = False
    else:
        print("   %d samples at a 100 Hz loop vs %d at 4 Hz: the stream follows its own rate, and at a"
              " loop slower than that rate one sample per pass is all there is to emit"
              % (len(d_cmds), len(a_cmds)))
    # The Path is published from the LOOP, so its publication COUNT is a function of the loop rate and
    # of where the arrival gate happened to fire -- 171 at 4 Hz against 4206 at 100 Hz is the same tour
    # described more finely, not a change caused by the stream.  What must not change is the tour's
    # CONTENT, and that is what is compared: the sequence of DISTINCT Path messages (consecutive repeats
    # collapsed, which is how a latched republish of the same leg reads) must be identical.
    def distinct(pubs):
        out = []
        for _, j in pubs:
            if not out or out[-1] != j:
                out.append(j)
        return out

    # The Path is published from the LOOP, so neither its publication COUNT nor its header stamps are
    # comparable across loop rates: 171 messages at 4 Hz and 4206 at 100 Hz are the same tour described
    # more finely, and the arrival gate fires at a different sub-metre offset because the vehicle moves in
    # steps of 0.30 m/s * (1/loop_rate).  What must not change is the tour's CONTENT.  So each Path
    # message's GEOMETRY is extracted (frame, every pose's position and quaternion -- the plan itself,
    # without the clock) and consecutive repeats are collapsed, which is how a latched republish of the
    # same leg reads: the same legs must come out of both runs.
    def plans(pubs):
        out = []
        for _, j in pubs:
            d = json.loads(j)
            geom = json.dumps({"frame_id": d["frame_id"],
                               "poses": [[p["frame_id"], p["p"], p["q"]] for p in d["poses"]]},
                              sort_keys=True)
            if not out or out[-1] != geom:
                out.append(geom)
        return out

    da, dd = distinct(a_paths), distinct(d_paths)
    pa, pd = plans(a_paths), plans(d_paths)
    print("   distinct Path messages (consecutive republishes collapsed): %d at 4 Hz, %d at 100 Hz"
          % (len(da), len(dd)))
    print("   the tour's distinct legs: %d at 4 Hz, %d at 100 Hz" % (len(pa), len(pd)))
    if pa != pd:
        n = min(len(pa), len(pd))
        first = next((i for i in range(n) if pa[i] != pd[i]), n)
        print("   FAIL the tour took a different SHAPE at the two loop rates: first difference at leg %d"
              % first)
        ok = False
    else:
        print("   the tour's Path CONTENT is identical at both loop rates: the same %d legs, commanded"
              " the same way, in the same order" % len(pa))
    if a_cmds:
        last = json.loads(a_cmds[-1][1])
        first = json.loads(a_cmds[0][1])
        print("   first command sample: flag=%d position=%s" % (first["trajectory_flag"],
                                                               first["position"]))
        print("   last  command sample: flag=%d position=%s velocity=%s"
              % (last["trajectory_flag"], last["position"], last["velocity"]))

    print()
    print("SHADOW A/B %s" % ("PASSED" if ok else "FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
