#!/usr/bin/env python3
"""The LIO's ATTITUDE against the truth -- the half of "accurate" that A1/A2 never read.

A1/A2 compare positions only.  A yaw or roll error that oscillates would swing the thrust direction and
show up as a cross-track wobble in B2 while every position metric stayed clean, so the attitude gets its
own number.

Frames: the tap's quaternion is body->NED on the wire and the LIO's is body-FLU->L, so comparing euler
angles directly is meaningless (the first version of this file did that and reported a 105 deg yaw rms:
two different conventions, not an error).  Instead the best-fit CONSTANT rotation L->W is solved for over
the window (Wahba, by SVD), the residual is reported as a rotation angle, and the fitted rotation is
printed -- it should reproduce the construction's +90.050 deg yaw, which makes this an independent check
of the calibration as well.

usage: tools/lio-attitude.py --tap <tap.csv> --odom <lio.csv> [--from-takeoff]
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, "/mnt/d/distill_self/lio_avoid")
from common import frames                       # noqa: E402
from tools.metrics import match                 # noqa: E402


def read_tap_pq(path):
    """(t, up, q_ned): the wire values, plus the height as a sign (up = -down), no frame crossing yet.

    The frame crossing happens once per matched sample through `frames.wire_to_world` (the single place
    the wire frame is crossed), not here: that function takes one sample at a time, and a tool that
    re-implemented the batch version would be a second place where the convention lives.
    """
    t, p, q = [], [], []
    with open(path) as fh:
        fh.readline()
        for line in fh:
            f = line.strip().split(",")
            if len(f) < 13:
                continue
            t.append(int(f[0]))
            p.append([float(f[2]), float(f[3]), float(f[4])])
            # the tap header is t_us,inst,pn,pe,pd,vn,ve,vd,qw,qx,qy,qz,wx,... so qw is field **8**.
            # Reading it as 9 gives (qx,qy,qz,wx) -- a well-formed tuple of the wrong four numbers, and
            # the fit then reports a 105 deg yaw error that is entirely my own indexing.
            q.append([float(f[8]), float(f[9]), float(f[10]), float(f[11])])
    p = np.array(p, dtype=float)
    return np.array(t, dtype=np.int64), -p[:, 2], np.array(q, dtype=float)


def read_odom_q(path):
    t, q = [], []
    with open(path) as fh:
        fh.readline()
        for line in fh:
            f = line.strip().split(",")
            if len(f) < 8:
                continue
            t.append(int(f[0]))
            q.append([float(f[4]), float(f[5]), float(f[6]), float(f[7])])
    return np.array(t, dtype=np.int64), np.array(q, dtype=float)


def rotvec_deg(R):
    """Rotation vector (axis * angle, degrees) of a rotation matrix, per sample."""
    tr = np.trace(R, axis1=-2, axis2=-1)
    c = np.clip((tr - 1.0) / 2.0, -1.0, 1.0)
    ang = np.arccos(c)
    small = ang < 1e-8
    s = np.where(small, 1.0, np.sin(ang))
    axis = np.stack([(R[..., 2, 1] - R[..., 1, 2]), (R[..., 0, 2] - R[..., 2, 0]),
                     (R[..., 1, 0] - R[..., 0, 1])], axis=-1) / (2.0 * s)[..., None]
    return np.degrees(axis * ang[..., None])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--tap", required=True)
    ap.add_argument("--odom", required=True)
    ap.add_argument("--from-takeoff", action="store_true", help="window = truth above 0.5 m")
    args = ap.parse_args()

    t_t, up_t, q_t_ned = read_tap_pq(args.tap)
    t_l, q_l = read_odom_q(args.odom)
    if args.from_takeoff:
        m = up_t > 0.5
        t_t, q_t_ned = t_t[m], q_t_ned[m]
    ia, ib = match(t_t, t_l)
    if len(ia) < 20:
        print("only %d matched samples" % len(ia))
        return 1
    q_t = np.array([frames.wire_to_world(np.zeros(3), np.zeros(3), q)[2] for q in q_t_ned[ia]])
    A = np.array([frames.R_from_q(q) for q in q_l[ib]])       # body -> L
    B = np.array([frames.R_from_q(q) for q in q_t])           # body -> W
    M = np.einsum("nij,nkj->ik", B, A)                        # sum B A^T
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    fitted = rotvec_deg(R)
    err = rotvec_deg(np.einsum("nji,njk->nik", B, np.einsum("ij,njk->nik", R, A)))
    tot = np.linalg.norm(err, axis=1)
    print("attitude over %d samples (%.1f s), LIO vs truth"
          % (len(ia), (t_t[ia][-1] - t_t[ia][0]) * 1e-6))
    print("   fitted constant rotation L->W: roll %+7.3f pitch %+7.3f yaw %+7.3f deg (angle %.3f)"
          % (*fitted, float(np.linalg.norm(fitted))))
    print("   residual total angle: rms %6.3f deg   p95 %6.3f   max %6.3f"
          % (np.sqrt(np.mean(tot ** 2)), np.percentile(tot, 95), tot.max()))
    for i, nm in enumerate(("body x", "body y", "body z")):
        e = err[:, i]
        print("   %-7s rms %6.3f deg   p-p %6.3f" % (nm, np.sqrt(np.mean(e ** 2)), e.max() - e.min()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
