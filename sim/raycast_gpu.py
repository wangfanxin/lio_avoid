#!/usr/bin/env python3
"""GPU ray casting for the pillar world, with the CPU implementation kept as the reference.

Design rules
  * ONE set of equations.  The CPU path in raycast.py and the GPU path here compute the same closed
    form (ray/plane, ray/cylinder); the benchmark asserts they agree, so a GPU bug cannot silently
    change the sensor model.
  * Batched by construction: the GPU pays off when many scans are cast at once (offline replays,
    parameter sweeps, Monte-Carlo of noise realisations, several vehicles).  `cast_batch()` takes
    (S, N) scan poses and returns (S, N) hits in one launch.
  * The primitive set is tiny (10 pillars + 2 planes + 4 walls), so the ray x primitive work is
    20 000 x 16 = 320 k tests per scan -- a rounding error for an RTX 4060, which is why the batch
    dimension, not the per-scan one, is where the headroom shows.

Usage:
    python3 raycast_gpu.py --bench                 # CPU vs GPU, 1 and 100 scans
    python3 raycast_gpu.py --verify                # element-wise agreement with the CPU path
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from raycast import World, load_pattern, make_mount_rotation, rpy_to_R, cast as cast_cpu  # noqa: E402

try:
    import cupy as cp
    HAVE_CUPY = True
except Exception:            # noqa: BLE001
    cp = None
    HAVE_CUPY = False

WORLD_DEFAULT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "worlds",
                             "pillars.yaml")
PATTERN_DEFAULT = os.path.join(os.path.dirname(os.path.abspath(__file__), ), "mid360_pattern.csv")


# ------------------------------------------------------------------ world -> flat primitive arrays
def flatten_world(w: World):
    """Return (planes, walls, cyls) as plain float arrays.

    planes: (P, 3) as (axis, coord, sign) -- axis 2 = horizontal plane, 0/1 = wall normal axis
    cyls:   (C, 4) as (cx, cy, r, z0, z1) padded to 5
    """
    planes = []
    if w.floor_z is not None:
        planes.append((2.0, float(w.floor_z), +1.0))
    if w.ceiling_z is not None:
        planes.append((2.0, float(w.ceiling_z), -1.0))
    walls = np.zeros((0, 4), dtype=np.float64)
    if w.walls:
        walls = np.array([
            [0.0, w.walls["x"][0], w.walls["y"][0], w.walls["y"][1]],
            [0.0, w.walls["x"][1], w.walls["y"][0], w.walls["y"][1]],
            [1.0, w.walls["y"][0], w.walls["x"][0], w.walls["x"][1]],
            [1.0, w.walls["y"][1], w.walls["x"][0], w.walls["x"][1]],
        ], dtype=np.float64)
    cyls = np.zeros((len(w.pillars), 5), dtype=np.float64)
    cyls[:, :w.pillars.shape[1]] = w.pillars
    return np.array(planes, dtype=np.float64).reshape(-1, 3), walls, cyls


def cast_batch_gpu(origins, Rs, rays, w: World, rmin=0.10, rmax=40.0, dtype=cp.float32 if HAVE_CUPY
                   else None):
    """origins (S,3), Rs (S,3,3) body->world, rays (N,2) sensor-frame az/el (already includes mount
    via Rs).  Returns (best_range (S,N), points (S,N,3)) with non-hits as rmax*2 / NaN."""
    planes, walls, cyls = flatten_world(w)
    o = cp.asarray(origins, dtype=dtype)
    R = cp.asarray(Rs, dtype=dtype)
    az = cp.asarray(rays[:, 0], dtype=dtype)[None, :]
    el = cp.asarray(rays[:, 1], dtype=dtype)[None, :]
    S = o.shape[0]
    N = az.shape[1]

    d_body = cp.stack([cp.cos(el) * cp.cos(az), cp.cos(el) * cp.sin(az), cp.sin(el)], axis=-1)
    d_body = cp.broadcast_to(d_body, (S, N, 3))
    d = cp.einsum("sij,snj->sni", R, d_body)                # (S,N,3) world directions
    oo = cp.broadcast_to(o[:, None, :], (S, N, 3))

    best = cp.full((S, N), cp.inf, dtype=dtype)

    # horizontal planes
    for axis, coord, sign in planes:
        axis = int(axis)
        if axis == 2:
            dd = d[..., 2]
            t = (coord - oo[..., 2]) / cp.where(cp.abs(dd) < 1e-12, cp.nan, dd)
            best = cp.where((t > 0) & (t < best), t, best)

    # walls: hit the plane, then check the other horizontal axis and the vertical span
    for axis, plane, lo, hi in walls:
        axis = int(axis)
        other = 1 - axis
        dd = d[..., axis]
        t = (plane - oo[..., axis]) / cp.where(cp.abs(dd) < 1e-12, cp.nan, dd)
        hit_other = oo[..., other] + t * d[..., other]
        ok = (t > 0) & (hit_other >= lo) & (hit_other <= hi)
        best = cp.where(ok & (t < best), t, best)

    # cylinders, vectorised over the (small) pillar list
    cx = cp.asarray(cyls[:, 0], dtype=dtype)[:, None, None]
    cy = cp.asarray(cyls[:, 1], dtype=dtype)[:, None, None]
    cr = cp.asarray(cyls[:, 2], dtype=dtype)[:, None, None]
    cz0 = cp.asarray(cyls[:, 3], dtype=dtype)[:, None, None]
    cz1 = cp.asarray(cyls[:, 4], dtype=dtype)[:, None, None]
    ox = oo[..., 0][None, :, :] - cx
    oy = oo[..., 1][None, :, :] - cy
    dx = d[..., 0][None, :, :]
    dy = d[..., 1][None, :, :]
    dz = d[..., 2][None, :, :]
    a = dx * dx + dy * dy
    b = 2.0 * (ox * dx + oy * dy)
    c = ox * ox + oy * oy - cr * cr
    disc = b * b - 4.0 * a * c
    with np.errstate(invalid="ignore"):
        sq = cp.sqrt(cp.where(disc > 0, disc, cp.nan))
    t1 = (-b - sq) / (2.0 * a)
    t2 = (-b + sq) / (2.0 * a)
    t = cp.where(t1 > 0, t1, t2)
    hz = oo[..., 2][None, :, :] + t * dz
    ok = (disc > 0) & (t > 0) & (hz >= cz0) & (hz <= cz1)
    cand = cp.where(ok, t, cp.inf)
    best = cp.minimum(best, cp.min(cand, axis=0))

    good = cp.isfinite(best) & (best >= rmin) & (best <= rmax)
    rng = cp.where(good, best, cp.nan)
    pts = oo + cp.where(cp.isnan(rng), 0, rng)[..., None] * d
    pts = cp.where(cp.isnan(rng)[..., None], cp.nan, pts)
    return rng, pts


# ------------------------------------------------------------------ benchmark
def bench(world_path: str, pattern_path: str, n_rays: int, batches, pos):
    w = World.load(world_path)
    rays, src = load_pattern(pattern_path, n_rays)
    x, y, z = pos
    mount = make_mount_rotation({"pitch": 45.0})
    R = rpy_to_R(0, 0, 0) @ mount
    print("world   : %d pillars, pattern %s (%d rays)" % (len(w.pillars), src, len(rays)))
    print("%-8s %-10s %10s %10s %12s" % ("scans", "backend", "total ms", "ms/scan", "max Hz"))
    for S in batches:
        origins = np.array([[x + 0.01 * i, y, z] for i in range(S)])
        Rs = np.broadcast_to(R, (S, 3, 3)).copy()
        # CPU
        t0 = time.perf_counter()
        for i in range(S):
            cast_cpu(origins[i], Rs[i], rays, w)
        dt_cpu = (time.perf_counter() - t0) / S
        # GPU
        dt_gpu = float("nan")
        if HAVE_CUPY:
            cast_batch_gpu(origins[:1], Rs[:1], rays, w)          # warm up / compile
            cp.cuda.Stream.null.synchronize()
            t0 = time.perf_counter()
            rng, pts = cast_batch_gpu(origins, Rs, rays, w)
            cp.cuda.Stream.null.synchronize()
            dt_gpu = (time.perf_counter() - t0) / S
        print("%-8d %-10s %10.1f %10.3f %12.0f" % (S, "cpu", dt_cpu * S * 1e3, dt_cpu * 1e3,
                                                   1.0 / dt_cpu))
        if HAVE_CUPY:
            print("%-8d %-10s %10.1f %10.3f %12.0f" % (S, "gpu", dt_gpu * S * 1e3, dt_gpu * 1e3,
                                                       1.0 / dt_gpu))
    return 0


def verify(world_path: str, pattern_path: str, n_rays: int, pos):
    if not HAVE_CUPY:
        print("cupy unavailable; nothing to verify")
        return 1
    w = World.load(world_path)
    rays, _ = load_pattern(pattern_path, n_rays)
    origin = np.array(pos)
    R = rpy_to_R(0, 0, 0) @ make_mount_rotation({"pitch": 45.0})
    pts_c, rng_c = cast_cpu(origin, R, rays, w)
    rng_g, pts_g = cast_batch_gpu(origin[None, :], R[None], rays, w)
    rng_g = cp.asnumpy(rng_g)[0]
    pts_g = cp.asnumpy(pts_g)[0]
    ok_c = np.isfinite(rng_c)
    ok_g = np.isfinite(rng_g)
    print("  hits      cpu %d   gpu %d   identical mask: %s"
          % (ok_c.sum(), ok_g.sum(), bool((ok_c == ok_g).all())))
    both = ok_c & ok_g
    if both.sum():
        dr = np.abs(rng_c[both] - rng_g[both])
        dp = np.nanmax(np.abs(pts_c[ok_c] - pts_g[ok_g])) if ok_c.sum() == ok_g.sum() else float("nan")
        print("  |drange|  max %.3e m  (float32 accumulation)" % dr.max())
        print("  |dpoint|  max %.3e m" % dp)
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bench", action="store_true")
    ap.add_argument("--verify", action="store_true")
    ap.add_argument("--world", default=WORLD_DEFAULT)
    ap.add_argument("--pattern", default=PATTERN_DEFAULT)
    ap.add_argument("--rays", type=int, default=20000)
    ap.add_argument("--pos", default="0,0,2.5")
    args = ap.parse_args()
    pos = tuple(float(v) for v in args.pos.split(","))
    print("cupy: %s" % ("available" if HAVE_CUPY else "MISSING"))
    if args.verify:
        return verify(args.world, args.pattern, args.rays, pos)
    if args.bench:
        return bench(args.world, args.pattern, args.rays, [1, 10, 100, 1000], pos)
    ap.print_help()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
