#!/usr/bin/env python3
"""Is the LIO's reported velocity the derivative of its reported position?

The bridge feeds `/lio/robo/odom` to the EKF as ODOMETRY, whose velocity field is BODY FRD.  If the LIO
actually publishes a WORLD velocity and the bridge rotates it as if it were body, the EKF gets a
velocity that contradicts the position it is also being given -- which drives the position loop and ends
in a runaway.  This compares, from one recording, the position derivative against the reported velocity
in (a) the world frame and (b) the body frame, and says which one it matches.

usage: tools/check-lio-velocity-frame.py <lio.csv>
"""
from __future__ import annotations

import argparse
import math
import sys

import numpy as np

sys.path.insert(0, "/mnt/d/distill_self/lio_avoid")
from common import frames          # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--limit", type=int, default=400)
    args = ap.parse_args()

    t, p, q, v = [], [], [], []
    with open(args.csv) as fh:
        fh.readline()
        for line in fh:
            f = line.strip().split(",")
            if len(f) < 14:
                continue
            t.append(int(f[0]))
            p.append([float(f[1]), float(f[2]), float(f[3])])
            q.append([float(f[4]), float(f[5]), float(f[6]), float(f[7])])
            v.append([float(f[8]), float(f[9]), float(f[10])])
    t = np.array(t, dtype=np.int64)
    p = np.array(p)
    q = np.array(q)
    v = np.array(v)
    print("%d samples over %.1f s" % (len(t), (t[-1] - t[0]) * 1e-6))

    # position derivative by central differences, over a window long enough to beat the noise
    step = max(1, len(t) // args.limit)
    idx = np.arange(step, len(t) - step, step)
    dt = (t[idx + step] - t[idx - step]) * 1e-6
    dp = (p[idx + step] - p[idx - step]) / dt[:, None]

    err_world = []
    err_body = []
    for k, i in enumerate(idx):
        R = frames.R_from_q(q[i])                    # body FLU -> L
        err_world.append(np.linalg.norm(dp[k] - v[i]))
        # the body-frame hypothesis: the reported v is the world velocity expressed in the body
        err_body.append(np.linalg.norm(dp[k] - R @ v[i]))
    err_world = np.array(err_world)
    err_body = np.array(err_body)
    print("|dp/dt - v|            (v is a WORLD vector): median %8.4f m/s, p90 %8.4f"
          % (float(np.median(err_world)), float(np.percentile(err_world, 90))))
    print("|dp/dt - R_body@v|     (v is a BODY vector) : median %8.4f m/s, p90 %8.4f"
          % (float(np.median(err_body)), float(np.percentile(err_body, 90))))
    print()
    if np.median(err_world) < np.median(err_body):
        print("=> the reported velocity is a WORLD vector (L), so rotating it by the attitude before")
        print("   sending it as body-FRD is correct")
    else:
        print("=> the reported velocity is a BODY vector, so the bridge must NOT rotate it again")
    print()
    print("speed range in the recording: %.3f .. %.3f m/s" % (float(np.min(np.linalg.norm(v, axis=1))),
                                                              float(np.max(np.linalg.norm(v, axis=1)))))
    return 0


if __name__ == "__main__":
    sys.exit(main())
