#!/usr/bin/env python3
"""Analytic ray casting for the pillar world -- no Gazebo, no meshes, no GPU.

Why analytic instead of a mesh ray tracer (trimesh/embree/pybullet):
  * the world is cylinders and planes, so the intersection is a closed-form quadratic; it is EXACT
    (no tessellation error) and vectorises trivially in numpy;
  * 20 000 rays at 10 Hz is 200 k intersections/s, which numpy does in a few milliseconds;
  * it removes a whole class of simulator defects that cost this project weeks before: the old
    Gazebo lidar quantised range to 2 cm and its physics step injected a phantom IMU acceleration.

Speed of light is not simulated: a "scan" is the set of hit points for a fixed ray pattern fired at
one instant, with per-point offsets assigned afterwards so the LIO can undistort.

Usage (self-test, no ROS needed):
    python3 raycast.py --world ../worlds/pillars.yaml --render /tmp/scan.png
    python3 raycast.py --world ../worlds/pillars.yaml --summary
"""
from __future__ import annotations

import argparse
import math
import os
import sys
from dataclasses import dataclass, field

import numpy as np

try:
    import yaml
except ImportError:                                    # keep the module usable without pyyaml
    yaml = None


# --------------------------------------------------------------------------- world
@dataclass
class World:
    floor_z: float = 0.0
    ceiling_z: float | None = 6.0
    walls: dict | None = None
    pad: dict | None = None
    pillars: np.ndarray = field(default_factory=lambda: np.zeros((0, 5)))

    # pillars: rows of (x, y, r, z0, z1)

    @staticmethod
    def load(path: str) -> "World":
        if yaml is None:
            raise RuntimeError("pyyaml is required to read %s" % path)
        with open(path) as fh:
            d = yaml.safe_load(fh)
        arena = d.get("arena", {}) or {}
        pl = d.get("pillars", []) or []
        arr = np.array([[p["x"], p["y"], p["r"], p.get("z0", 0.0), p.get("z1", 4.0)] for p in pl],
                       dtype=float) if pl else np.zeros((0, 5))
        return World(floor_z=float(arena.get("floor_z", 0.0)),
                     ceiling_z=(None if arena.get("ceiling_z") is None
                                else float(arena["ceiling_z"])),
                     walls=arena.get("walls"), pad=d.get("pad"), pillars=arr)

    def bounds(self):
        if self.walls:
            return (self.walls["x"][0], self.walls["x"][1], self.walls["y"][0], self.walls["y"][1])
        if len(self.pillars):
            return (self.pillars[:, 0].min() - 2, self.pillars[:, 0].max() + 2,
                    self.pillars[:, 1].min() - 2, self.pillars[:, 1].max() + 2)
        return (-6.0, 6.0, -6.0, 6.0)


# --------------------------------------------------------------------------- lidar pattern
S = "/mnt/d/distill_self/_stage"      # kept for the older tools that import from here
MID360_POINT_RATE_HZ = 200000.0       # the real sensor's point rate: 200 000 points/s


def synth_pattern(n: int = 20000, az_per_rev: int = 360, seed: int = 0) -> np.ndarray:
    """Fallback MID-360-like pattern: a non-repetitive rosette.

    The real MID-360 fires along a fixed but non-repeating rosette; its CSV of ray directions is
    public (Livox-SDK/livox_laser_simulation scan_mode/mid360.csv) and is preferred when present.
    This synthesises something with the same coverage (360 deg azimuth, -7..+52 deg elevation) so
    the geometry can be exercised before that file is available.
    """
    rng = np.random.default_rng(seed)
    hfov = 360.0
    vlo, vhi = -7.0, 52.0
    k = np.arange(n, dtype=float)
    az = (k * hfov / az_per_rev) % 360.0                     # fast azimuth sweep
    # elevation walks upward across the scan, with a small per-ray jitter -> non-repetitive
    elev = vlo + (vhi - vlo) * ((k % 400.0) / 400.0) + rng.normal(0.0, 0.02, n)
    return np.column_stack([np.radians(az), np.radians(elev)])


def load_pattern(path: str | None, n: int = 20000, start: int = 0):
    """Return (rays, times, source).

    rays  : (N, 2) azimuth / elevation in radians, in the SENSOR frame
    times : (N,)   per-point offset within the scan, seconds (None if not available)
    source: a human-readable description of where the pattern came from

    Two formats are supported, and the difference matters:
      * the REAL MID-360 pattern from Livox-SDK/livox_laser_simulation,
        `scan_mode/mid360.csv`, header `Time/s,Azimuth/deg,Zenith/deg`, 800 001 rows = one second at
        200 kHz.  ZENITH is the angle from +z, so elevation = 90 - zenith.  Reading that file as if
        the second column were elevation -- the obvious mistake -- puts the fan in the wrong half of
        the sky.  The file's own time column is kept as the per-point timestamp (5 us spacing), which
        is more honest than assuming a linear ramp.
      * a synthesised rosette, used only when the file is absent.

    `start` lets consecutive scans use consecutive windows of the pattern, which is how the real
    non-repetitive sensor behaves (the pattern advances through the second).
    """
    if path and os.path.exists(path):
        try:
            with open(path) as fh:
                head = fh.readline()
            if "Zenith" in head:
                a = np.loadtxt(path, delimiter=",", skiprows=1 + start, usecols=(0, 1, 2),
                               max_rows=n)
                if a.ndim == 2 and len(a) > 100:
                    t = a[:, 0] - a[0, 0]
                    # The column is HEADERED "Time/s" but is in fact a sample index (step exactly 1,
                    # spanning 800 000 over the file).  Dividing by the sensor's point rate turns it
                    # into the per-point offset in seconds -- 5 us spacing, 100 ms for 20 000 points,
                    # which is exactly one scan at 10 Hz.  Detect it rather than assume it.
                    step = float(np.median(np.diff(t))) if len(t) > 1 else 0.0
                    if step > 0.5:                       # index, not seconds
                        t = t / MID360_POINT_RATE_HZ
                    az = np.radians(a[:, 1])
                    el = np.radians(90.0 - a[:, 2])
                    return np.column_stack([az, el]), t, "%s (real MID-360, rows %d..%d)" % (
                        os.path.basename(path), start + 1, start + len(a))
            a = np.loadtxt(path, delimiter=",", skiprows=1, usecols=(0, 1))
            if a.ndim == 2 and len(a) > 100:
                if len(a) > n:                          # thin uniformly, keep the order
                    idx = np.linspace(0, len(a) - 1, n).astype(int)
                    a = a[idx]
                return np.radians(a.astype(float)), None, os.path.basename(path)
        except Exception as exc:                        # noqa: BLE001
            print("  pattern file unreadable (%s); synthesising instead" % exc, file=sys.stderr)
    return synth_pattern(n), None, "synthesised rosette"


# --------------------------------------------------------------------------- ray casting
def cast(origin: np.ndarray, R: np.ndarray, rays: np.ndarray, world: World, *,
         rmin: float = 0.10, rmax: float = 40.0, return_full: bool = False):
    """Cast `rays` (N,2: azimuth, elevation) from `origin` with attitude `R` (body->world).

    The sensor frame is the body frame rotated by the mount, already folded into R by the caller.
    Returns (points_world (M,3), ranges (M,)) with non-hits dropped; with return_full=True it also
    returns best_t (N,) where misses are +inf, which is what the self-test compares against.
    """
    n = len(rays)
    az, el = rays[:, 0], rays[:, 1]
    d_body = np.column_stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    d = d_body @ R.T                                           # to world
    o = np.broadcast_to(origin, d.shape)

    best_t = np.full(n, np.inf)

    # --- horizontal planes: floor and ceiling
    for z_plane in (world.floor_z, world.ceiling_z):
        if z_plane is None:
            continue
        dz = d[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (z_plane - o[0, 2]) / dz
        ok = np.isfinite(t) & (t > 0)
        best_t = np.where(ok & (t < best_t), t, best_t)

    # --- axis-aligned walls (inside faces)
    if world.walls:
        for axis, (lo, hi) in enumerate((world.walls["x"], world.walls["y"])):
            for plane, sign in ((lo, +1.0), (hi, -1.0)):
                dd = d[:, axis]
                with np.errstate(divide="ignore", invalid="ignore"):
                    t = (plane - o[0, axis]) / dd
                ok = np.isfinite(t) & (t > 0)
                hit = o + t[:, None] * d
                # only count it if the hit is inside the box on the other horizontal axis
                other = 1 - axis
                olo, ohi = (world.walls["x"] if other == 0 else world.walls["y"])
                ok &= (hit[:, other] >= olo) & (hit[:, other] <= ohi)
                if world.ceiling_z is not None:
                    ok &= (hit[:, 2] >= world.floor_z) & (hit[:, 2] <= world.ceiling_z)
                best_t = np.where(ok & (t < best_t), t, best_t)

    # --- vertical cylinders
    #
    # MEASURED 2026-10-13: an azimuth-window culling of this loop (only test rays whose world azimuth
    # falls inside a pillar's apparent angular width) FAILED its own self-test on 5 of 6 poses -- it
    # missed near pillars, so a ray that should have been rejected as inside the blind zone was kept
    # at the farther hit.  It was also SLOWER than this plain loop (22.3 ms vs 18.2 ms at 48
    # pillars), because the mask costs more than the vectorised quadratic it saves.  So: no culling.
    # The scalar reference in brute_force() is what keeps this honest.
    for cx, cy, r, z0, z1 in world.pillars:
        ox, oy = o[0, 0] - cx, o[0, 1] - cy
        dx, dy = d[:, 0], d[:, 1]
        a = dx * dx + dy * dy
        b = 2.0 * (ox * dx + oy * dy)
        c = ox * ox + oy * oy - r * r
        disc = b * b - 4.0 * a * c
        ok = (disc > 0) & (a > 1e-12)
        if not np.any(ok):
            continue
        sq = np.sqrt(np.where(ok, disc, 0.0))
        t1 = (-b - sq) / (2.0 * a)
        t2 = (-b + sq) / (2.0 * a)
        t = np.where(t1 > 0, t1, t2)
        hit_z = o[0, 2] + t * d[:, 2]
        ok &= (t > 0) & (hit_z >= z0) & (hit_z <= z1)
        best_t = np.where(ok & (t < best_t), t, best_t)

    good = np.isfinite(best_t) & (best_t >= rmin) & (best_t <= rmax)
    t = best_t[good]
    pts = o[good] + t[:, None] * d[good]
    if return_full:
        return pts, t, best_t
    return pts, t


def cast_full(origin: np.ndarray, R: np.ndarray, rays: np.ndarray, world: World, *,
              rmin: float = 0.10, rmax: float = 40.0) -> tuple[np.ndarray, np.ndarray]:
    """Reference implementation: every ray tested against every pillar, no culling.

    Only used by --selftest to prove the culled path in cast() returns the same hits.  A dense forest
    makes this the slow one on purpose.
    """
    n = len(rays)
    az, el = rays[:, 0], rays[:, 1]
    d_body = np.column_stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    d = d_body @ R.T
    o = np.broadcast_to(origin, d.shape)
    best_t = np.full(n, np.inf)
    for z_plane in (world.floor_z, world.ceiling_z):
        if z_plane is None:
            continue
        dz = d[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (z_plane - o[0, 2]) / dz
        ok = np.isfinite(t) & (t > 0)
        best_t = np.where(ok & (t < best_t), t, best_t)
    for cx, cy, r, z0, z1 in world.pillars:
        ox, oy = o[0, 0] - cx, o[0, 1] - cy
        dx, dy = d[:, 0], d[:, 1]
        a = dx * dx + dy * dy
        b = 2.0 * (ox * dx + oy * dy)
        c = ox * ox + oy * oy - r * r
        disc = b * b - 4.0 * a * c
        ok = (disc > 0) & (a > 1e-12)
        if not np.any(ok):
            continue
        sq = np.sqrt(np.where(ok, disc, 0.0))
        t1 = (-b - sq) / (2.0 * a)
        t2 = (-b + sq) / (2.0 * a)
        t = np.where(t1 > 0, t1, t2)
        hit_z = o[0, 2] + t * d[:, 2]
        ok &= (t > 0) & (hit_z >= z0) & (hit_z <= z1)
        best_t = np.where(ok & (t < best_t), t, best_t)
    good = np.isfinite(best_t) & (best_t >= rmin) & (best_t <= rmax)
    t = best_t[good]
    return o[good] + t[:, None] * d[good], t


def brute_force(origin: np.ndarray, R: np.ndarray, rays: np.ndarray, world: World,
                limit: int = 400) -> np.ndarray:
    """Scalar, per-ray, per-primitive reference -- independent code, independent maths.

    Deliberately plain Python with math.* calls so a bug in the vectorised version cannot hide behind
    a shared mistake.  Returns the per-ray nearest-hit range for the first `limit` rays, +inf for a
    true miss -- and, like cast(..., return_full=True), WITHOUT the blind-zone/max-range gate, so the
    two can be compared directly.  (An earlier version of this test gated one side and not the other,
    which made 208 of 400 rays look like disagreements when both were right.)
    """
    pts, rngs = [], []
    out = np.full(min(limit, len(rays)), np.inf)
    for i in range(min(limit, len(rays))):
        az, el = float(rays[i, 0]), float(rays[i, 1])
        db = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
        d = R @ db
        best = math.inf
        for z_plane in (world.floor_z, world.ceiling_z):
            if z_plane is None or abs(d[2]) < 1e-15:
                continue
            t = (z_plane - origin[2]) / d[2]
            if t > 0:
                best = min(best, t)
        if world.walls:
            for axis, (lo, hi) in enumerate((world.walls["x"], world.walls["y"])):
                for plane in (lo, hi):
                    if abs(d[axis]) < 1e-15:
                        continue
                    t = (plane - origin[axis]) / d[axis]
                    if t <= 0 or t >= best:
                        continue
                    h = origin + t * d
                    other = 1 - axis
                    olo, ohi = (world.walls["x"] if other == 0 else world.walls["y"])
                    if olo <= h[other] <= ohi and (world.ceiling_z is None or
                                                   world.floor_z <= h[2] <= world.ceiling_z):
                        best = t
        for cx, cy, r, z0, z1 in world.pillars:
            ox, oy = origin[0] - cx, origin[1] - cy
            a = d[0] * d[0] + d[1] * d[1]
            if a < 1e-15:
                continue
            b = 2.0 * (ox * d[0] + oy * d[1])
            c = ox * ox + oy * oy - r * r
            disc = b * b - 4.0 * a * c
            if disc <= 0:
                continue
            sq = math.sqrt(disc)
            for t in ((-b - sq) / (2.0 * a), (-b + sq) / (2.0 * a)):
                if t > 0 and t < best:
                    hz = origin[2] + t * d[2]
                    if z0 <= hz <= z1:
                        best = t
                        break
        out[i] = best if math.isfinite(best) else math.inf
        if math.isfinite(best):
            pts.append(origin + best * d)
            rngs.append(best)
    return out


def make_mount_rotation(mount: dict) -> np.ndarray:
    """Body <- sensor rotation from the yaml mount (degrees)."""
    r, p, y = (math.radians(mount.get(k, 0.0)) for k in ("roll", "pitch", "yaw"))
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1.0]])
    Ry = np.array([[cp, 0, sp], [0, 1.0, 0], [-sp, 0, cp]])
    Rx = np.array([[1.0, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def rpy_to_R(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


# --------------------------------------------------------------------------- self-test
def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default=os.path.join(os.path.dirname(__file__),
                                                    "..", "worlds", "pillars.yaml"))
    ap.add_argument("--render", default=None, help="write a PNG of one scan")
    ap.add_argument("--summary", action="store_true")
    ap.add_argument("--selftest", action="store_true",
                    help="prove the culled casting equals the un-culled reference")
    ap.add_argument("--scale", action="store_true",
                    help="timing versus pillar count (the forest gets dense)")
    ap.add_argument("--pos", default="0,0,3.2", help="sensor origin x,y,z")
    ap.add_argument("--pattern", default=os.path.join(os.path.dirname(__file__),
                                                      "mid360_pattern.csv"))
    args = ap.parse_args()

    w = World.load(args.world)
    rays, times, src = load_pattern(args.pattern)
    x, y, z = (float(v) for v in args.pos.split(","))
    cfg = {}
    if yaml is not None:
        cfg = yaml.safe_load(open(args.world)) or {}
    mount = (cfg.get("sensor", {}) or {}).get("mount", {}) or {}

    if args.selftest:
        rng = np.random.default_rng(1)
        bad = 0
        for trial in range(6):
            origin = np.array([rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(0.5, 3.5)])
            R = rpy_to_R(rng.uniform(-0.3, 0.3), rng.uniform(-0.3, 0.3), rng.uniform(-np.pi, np.pi))
            _, _, vec = cast(origin, R, rays, w, return_full=True)
            bru = brute_force(origin, R, rays, w, limit=400)
            n = len(bru)
            a, b = vec[:n], bru
            same_mask = np.isfinite(a) == np.isfinite(b)
            same_val = np.allclose(np.where(np.isfinite(a), a, 0), np.where(np.isfinite(b), b, 0),
                                   atol=1e-9)
            hit_v, hit_b = int(np.isfinite(a).sum()), int(np.isfinite(b).sum())
            ok = bool(same_mask.all() and same_val)
            print("  trial %d: origin (%+.2f, %+.2f, %+.2f)  %d-ray sample: vector %d hits, "
                  "brute force %d hits, max |drange| %.2e m, identical=%s"
                  % (trial, origin[0], origin[1], origin[2], n, hit_v, hit_b,
                     float(np.max(np.abs(np.where(np.isfinite(a), a, 0)
                                         - np.where(np.isfinite(b), b, 0)))), ok))
            bad += 0 if ok else 1
        # and the gated output must drop exactly the sub-blind-zone hits
        _, rng = cast(origin, R, rays, w)
        print("  gate check at the last trial: %d of %d finite ranges survive the blind-zone gate"
              % (len(rng), int(np.isfinite(vec).sum())))
        print("  SELFTEST %s" % ("PASS" if bad == 0 else "FAIL (%d of 6 differ)" % bad))
        return 0 if bad == 0 else 1

    if args.scale:
        import time
        print("  pillars   rays   culled ms   full ms   speedup")
        for P in (10, 25, 49, 100, 200, 400):
            ww = World(floor_z=w.floor_z, ceiling_z=w.ceiling_z, walls=w.walls, pad=w.pad,
                       pillars=np.array([[np.cos(2 * np.pi * i / P) * 3.0,
                                          np.sin(2 * np.pi * i / P) * 3.0, 0.3, 0.0, 4.0]
                                         for i in range(P)]))
            R = rpy_to_R(0, 0, 0) @ make_mount_rotation(mount)
            o = np.array([0.0, 0.0, 2.5])
            cast(o, R, rays, ww)
            t0 = time.perf_counter()
            for _ in range(10):
                cast(o, R, rays, ww)
            tc = (time.perf_counter() - t0) / 10
            t0 = time.perf_counter()
            for _ in range(3):
                cast_full(o, R, rays, ww)
            tf = (time.perf_counter() - t0) / 3
            print("  %7d %6d %10.2f %9.2f %8.1fx" % (P, len(rays), tc * 1e3, tf * 1e3, tf / tc))
        return 0

    # The lidar's fan is defined about the SENSOR axis, and the sensor keeps the real drone's
    # 45 deg nose-down mount, so R_world_sensor = R_world_body @ R_body_sensor.
    R = rpy_to_R(0.0, 0.0, 0.0) @ make_mount_rotation(mount)
    pts, rng = cast(np.array([x, y, z]), R, rays, w)
    print("world   : %d pillars, floor %.1f, ceiling %s, walls %s"
          % (len(w.pillars), w.floor_z, w.ceiling_z, bool(w.walls)))
    print("pattern : %s (%d rays)" % (src, len(rays)))
    if times is not None and len(times) > 1:
        print("timing  : per-point offsets %.6f .. %.6f s (median step %.1f us)"
              % (times[0], times[-1], 1e6 * float(np.median(np.diff(times)))))
    print("mount   : roll %+.1f pitch %+.1f yaw %+.1f deg  (sensor fan becomes %+.0f..%+.0f deg "
          "about the body)" % (mount.get("roll", 0.0), mount.get("pitch", 0.0), mount.get("yaw", 0.0),
                               -7.0 - mount.get("pitch", 0.0), 52.0 - mount.get("pitch", 0.0)))
    print("origin  : (%.2f, %.2f, %.2f)  body level" % (x, y, z))
    print("hits    : %d of %d rays (%.1f %%)" % (len(pts), len(rays), 100.0 * len(pts) / len(rays)))
    if len(pts):
        print("range   : %.2f .. %.2f m (median %.2f)" % (rng.min(), rng.max(), np.median(rng)))
        # which primitive each hit came from is not tracked; report the geometric split instead
        on_floor = np.isclose(pts[:, 2], w.floor_z, atol=1e-6)
        on_ceil = (w.ceiling_z is not None) and np.isclose(pts[:, 2], w.ceiling_z, atol=1e-6)
        print("split   : floor %.1f %%, ceiling %.1f %%, other (pillars/walls) %.1f %%"
              % (100.0 * on_floor.mean(), 100.0 * on_ceil.mean() if w.ceiling_z else 0.0,
                 100.0 * (1 - on_floor.mean() - (on_ceil.mean() if w.ceiling_z else 0.0))))
    if args.render:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        x0, x1, y0, y1 = w.bounds()
        fig, ax = plt.subplots(1, 2, figsize=(13, 5.4))
        ax[0].scatter(pts[:, 0], pts[:, 1], s=0.4, c=pts[:, 2], cmap="viridis")
        for cx, cy, r, _, _ in w.pillars:
            ax[0].add_patch(plt.Circle((cx, cy), r, color="k", fill=False, lw=1.2))
        ax[0].plot(x, y, "r*", ms=14, label="sensor")
        ax[0].set_xlim(x0, x1); ax[0].set_ylim(y0, y1); ax[0].set_aspect("equal")
        ax[0].set_xlabel("East (m)"); ax[0].set_ylabel("North (m)")
        ax[0].set_title("one scan, top view (%d points)" % len(pts))
        ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3)
        ax[1].scatter(pts[:, 0], pts[:, 2], s=0.4, c=pts[:, 1], cmap="magma")
        ax[1].set_xlabel("East (m)"); ax[1].set_ylabel("Up (m)")
        ax[1].set_title("front view (colour = North)")
        ax[1].grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(args.render, dpi=140)
        print("render  : %s" % args.render)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
