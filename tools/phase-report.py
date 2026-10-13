#!/usr/bin/env python3
"""What actually happened physically?  Print, per recording, the truth's and the LIO's extent and
heading at start/end -- so the fits are interpreted against the motion that really occurred.

usage: tools/phase-report.py --tap logs/s6-lio-*/tap.csv --odom <csv> [--label parked]
"""
from __future__ import annotations

import argparse
import math
import sys

import numpy as np

sys.path.insert(0, "/mnt/d/distill_self/lio_avoid")
from common import frames          # noqa: E402


def read_tap(path):
    t, p, q = [], [], []
    with open(path) as fh:
        fh.readline()
        for line in fh:
            f = line.strip().split(",")
            if len(f) < 14:
                continue
            t.append(int(f[0]))
            p.append([float(f[2]), float(f[3]), float(f[4])])
            q.append([float(f[7]), float(f[8]), float(f[9]), float(f[10])])
    return np.array(t, dtype=np.int64), np.array(p, dtype=float), np.array(q, dtype=float)


def read_odom(path):
    t, p, q = [], [], []
    with open(path) as fh:
        fh.readline()
        for line in fh:
            f = line.strip().split(",")
            if len(f) < 14:
                continue
            t.append(int(f[0]))
            p.append([float(f[1]), float(f[2]), float(f[3])])
            q.append([float(f[4]), float(f[5]), float(f[6]), float(f[7])])
    return np.array(t, dtype=np.int64), np.array(p, dtype=float), np.array(q, dtype=float)


def extent(p):
    c = p - p.mean(axis=0)
    # normalise: the raw singular values of an (N,3) cloud scale with sqrt(N)
    sv = np.linalg.svd(c, compute_uv=False) / math.sqrt(max(1, len(c)))
    return sv, float(np.max(np.linalg.norm(c, axis=1)))


def yaw_of(q):
    """Yaw of a body->world quaternion, degrees, in the ENU sense."""
    R = frames.R_from_q(q)
    return math.degrees(math.atan2(R[1, 0], R[0, 0]))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--tap", required=True)
    ap.add_argument("--odom", required=True)
    ap.add_argument("--label", default="phase")
    args = ap.parse_args()

    t_tap, p_ned, q_ned = read_tap(args.tap)
    t_lio, p_lio, q_lio = read_odom(args.odom)
    t0, t1 = t_lio[0], t_lio[-1]
    keep = (t_tap >= t0) & (t_tap <= t1)
    tw = t_tap[keep]
    # truth: NED -> ENU, and the body attitude in W
    p_w = np.array([frames.ned_to_enu(x) for x in p_ned[keep]])
    q_w = np.array([frames.wire_to_world([0, 0, 0], [0, 0, 0], q)[2] for q in q_ned[keep]])

    print("== %s ==" % args.label)
    print("   window            %.2f s of sim time" % ((t1 - t0) * 1e-6))
    sv_t, max_t = extent(p_w)
    sv_l, max_l = extent(p_lio)
    print("   TRUTH in W        sv %.4f %.4f %.4f m, max excursion %.4f m" % (sv_t[0], sv_t[1],
                                                                             sv_t[2], max_t))
    print("   LIO in L          sv %.4f %.4f %.4f m, max excursion %.4f m" % (sv_l[0], sv_l[1],
                                                                             sv_l[2], max_l))
    print("   truth start pos   [%+.4f %+.4f %+.4f] m (W)" % tuple(p_w[0]))
    print("   truth end pos     [%+.4f %+.4f %+.4f] m (W)" % tuple(p_w[-1]))
    print("   truth travelled   %.4f m" % float(np.sum(np.linalg.norm(np.diff(p_w, axis=0), axis=1))))
    print("   truth yaw         %+.2f -> %+.2f deg" % (yaw_of(q_w[0]), yaw_of(q_w[-1])))
    print("   truth yaw span    %+.2f deg (max-min over the window)"
          % (float(np.max([yaw_of(q) for q in q_w]) - np.min([yaw_of(q) for q in q_w]))))
    print("   truth altitude    %.4f -> %.4f m" % (float(p_w[0][2]), float(p_w[-1][2])))
    return 0


if __name__ == "__main__":
    sys.exit(main())
