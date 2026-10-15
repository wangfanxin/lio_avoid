#!/usr/bin/env python3
"""Record what the dropped planner ITSELF published -- `planning/bspline` -- one row per message.

STATUS: KEPT, NEEDS A DECISION.  This is an instrument of the ego-planner line, which the owner dropped
on 2026-10-15; it is in the cleanup's keep list (`record-*.py`) but nothing in the Mission-A pipeline
produces `planning/bspline`, so nothing invokes it any more.  It needs the message type `traj_utils/Bspline`
from the planner's own ROS workspace to import at all, and that workspace lived OUTSIDE this repository --
it is no longer built, sourced or referenced by anything here.  Kept because it is the only record of the
measurement below; delete it if the record is not wanted.

WHY IT EXISTED.  MEASURED 2026-10-14 (sortie F1): the aircraft hovered on the pad for the whole 60 s the
first stop was commanded, and the FSM never received a single plan.  The published-path recorder showed one
plan withheld with the SAME start_time from t=16.7 s to t=66.7 s -- a stale trajectory offered over and
over, with no new one committed -- and the planner's own internal FSM logged `from REPLAN_TRAJ to
REPLAN_TRAJ` 5031 times (10 ms tick, so 50.3 s of consecutive local-replan failures; upstream retries
silently: ego_replan_fsm.cpp:583-586).  None of that was visible in the run dir, because nothing recorded
the planner's own output.  This file records it.

One row per message:
    t_recv_us  when we received it (system clock; the same clock /planner/traj is stamped with)
    drone_id, order, traj_id   straight from the message
    start_us   the trajectory's own start stamp (the planner's timeline)
    n_cp, n_kn  control points and knots
    k0, k1, k_last             the first two and the last knot -- k1-k0 is the time step ts
    cp0_*, cp1_*, cpN_*        the first two and the last control points (L frame)
    yaw_n, yaw_dt              the yaw profile's size (it was never flown; recorded for the record)

plus `<csv>.points.csv`: every control point, tagged with t_recv_us and traj_id.

A NEW traj_id is what "a new trajectory was committed" looks like; a start_us that never advances is what
"the planner is stalled" looks like.  Both were inferred from prose in F1.

usage: tools/record-bspline.py --csv out.csv [--topic planning/bspline] [--seconds N]
"""
from __future__ import annotations

import argparse
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="planning/bspline")
    ap.add_argument("--csv", required=True)
    ap.add_argument("--seconds", type=float, default=0.0, help="0 = until Ctrl-C")
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

    try:
        from traj_utils.msg import Bspline
    except Exception as exc:                                  # noqa: BLE001
        print("cannot import traj_utils/Bspline (%s).  That message type belongs to the DROPPED planner\n"
              "chain, whose workspace is not built any more; nothing in the Mission-A pipeline needs it."
              % exc, file=sys.stderr)
        return 2

    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    except Exception:                                         # noqa: BLE001
        rclpy.init(args=[])
    node = Node("record_bspline")
    # ego publishes planning/bspline as a normal (volatile) topic at the replan rate (10-100 Hz when it is
    # healthy, 0 Hz when it is stalled), so a deep RELIABLE queue on our side is enough to see every commit.
    qos = QoSProfile(depth=200, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE)
    fh = open(args.csv, "w")
    fh.write("t_recv_us,drone_id,order,traj_id,start_us,n_cp,n_kn,k0,k1,k_last,"
             "cp0_e,cp0_n,cp0_u,cp1_e,cp1_n,cp1_u,cpN_e,cpN_n,cpN_u,yaw_n,yaw_dt\n")
    pf = open(args.csv + ".points.csv", "w")
    pf.write("t_recv_us,traj_id,i,e,n,u\n")
    state = {"n": 0, "ids": set()}

    def on_bspline(m) -> None:
        t = int(node.get_clock().now().nanoseconds // 1000)
        start = int(m.start_time.sec) * 1_000_000 + int(m.start_time.nanosec) // 1000
        cp = [[float(p.x), float(p.y), float(p.z)] for p in m.pos_pts]
        kn = [float(k) for k in m.knots]
        j = lambda v: ",".join("%.6f" % x for x in v)                       # noqa: E731
        nan3 = [float("nan")] * 3
        fh.write("%d,%d,%d,%d,%d,%d,%d,%.6f,%.6f,%.6f,%s,%s,%s,%d,%.6f\n" % (
            t, int(m.drone_id), int(m.order), int(m.traj_id), start, len(cp), len(kn),
            kn[0] if kn else float("nan"),
            kn[1] if len(kn) > 1 else float("nan"),
            kn[-1] if kn else float("nan"),
            j(cp[0] if cp else nan3), j(cp[1] if len(cp) > 1 else nan3), j(cp[-1] if cp else nan3),
            len(m.yaw_pts), float(m.yaw_dt)))
        for i, p in enumerate(cp):
            pf.write("%d,%d,%d,%.6f,%.6f,%.6f\n" % (t, int(m.traj_id), i, p[0], p[1], p[2]))
        fh.flush()
        pf.flush()
        state["n"] += 1
        state["ids"].add(int(m.traj_id))

    node.create_subscription(Bspline, args.topic, on_bspline, qos)
    print("recording %s -> %s" % (args.topic, args.csv), flush=True)
    t_start = time.monotonic()
    try:
        while args.seconds == 0.0 or (time.monotonic() - t_start) < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    fh.close()
    pf.close()
    print("recorded %d bspline messages, %d distinct traj_id" % (state["n"], len(state["ids"])))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
