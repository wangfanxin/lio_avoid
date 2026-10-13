# STATUS — where the project stands

Living document 4 of 4 (budget: `PLAN.md` §6).  Decisions live in `PLAN.md` §2, the architecture in
`ARCHITECTURE.md`, the frames in `FRAMES.md`.  This file is only: what is done, with evidence, and what
is next.

## Done, with evidence

| # | item | evidence |
|---|---|---|
| 1 | old trees cleaned: `/home/fanxin/gazebo_apm` (9.3 G), `/home/fanxin/ardupilot` (3.3 G), `_stage` (86 M) | 13 G freed; keep-list of 22 M / 736 files in `/mnt/d/distill_self/_keep` |
| 2 | new skeleton `~/lio_avoid` (runtime) + `D:\distill_self\lio_avoid` (repo, docs live here) | `sim/ bridge/ common/ fsm/ planner/ worlds/ tools/ docs/ patches/ params/` |
| 3 | fresh clones | Super-LIO @ `f89f48d` (ros2), ikd-Tree @ `c0e36a1`, ArduPilot **Copter-4.7.1** from the gitee mirror (734 M, 7745 files).  `modules/mavlink` is still EMPTY -> the build needs `git submodule update --init --depth 1 modules/mavlink` |
| 4 | analytic ray caster | `sim/raycast.py --selftest` PASS: vectorised vs an independent scalar reference, 6 poses, max range difference **2e-14 m** |
| 5 | real MID-360 pattern | `scan_mode/mid360.csv`: azimuth 0.04-359.95 deg, elevation -7.16..+52.16 deg, 5 us/point (200 kHz), non-repetitive (consecutive windows differ by up to 278 deg).  The `Time/s` column is actually a sample index -- detected and handled |
| 6 | the world | `worlds/pillars.yaml`: 46 pillars r = 0.30 m, 1 m cells with integer boundaries, 0.8 m corridors (measured min 0.814 m), the six cells x[0,3] y[0,2] EMPTY (verified two ways), pad clearance 1.281 m, square x[0.5,2.5] y[0.4,1.6] with 0.600 m worst pillar clearance |
| 7 | single-frame scan | 0.5 m altitude: 20 000 points, 100 % returns, range 0.66-8.07 m, median 2.86 m, **floor 23.5 % / ceiling 16.6 % / pillars+walls 59.9 %**; 2.5 m for comparison: floor 6.8 % / ceiling 33.9 %.  Cost 9.8 ms/scan = 103 Hz capable.  Figures in `docs/figures/scan_at_low_real.png` |
| 8 | frame conventions researched from source | ArduPilot Copter-4.7.1: setpoints accept only LOCAL_NED / LOCAL_OFFSET_NED / BODY_NED / BODY_OFFSET_NED; `ODOMETRY` is silently DROPPED unless `frame_id = LOCAL_FRD` and `child_frame_id = BODY_FRD`; mavros does ENU->NED itself in `vision_pose_estimate.cpp:133-139` and stamps LOCAL_FRD/BODY_FRD in `odom.cpp:295-296` |
| 9 | documentation budget | 4 of 5 used, enforced by `tools/docs-budget.py` |
| 10 | **the "fork" was not a fork** | diffed our old local Super-LIO copy against the clean upstream clone (`f89f48d`): the code differences are NOT our work -- the local copy is an OLDER upstream snapshot.  Upstream has since (a) given `RightJacobianSO3` finite-value guards and a Taylor branch near theta -> 0, and (b) FIXED the undistortion, which in our copy advances each point by the whole interval `dt` instead of the fractional `tau`.  Consequence: **the round-7 accuracy numbers (3D 5.0-6.4 mm) were measured with the older code**; the new stack builds the clean upstream and carries over only our measured configuration (`config/livox_360.yaml`, kept in `src/lio-fork-local/config/`).  A junk 0-byte file literally named `*.h` (a bad shell redirection in an old session) was removed from that copy |

## Next, in order

1. **SITL truth tap**: `modules/mavlink` submodule, then the ~30-line env-gated truth broadcast in
   `libraries/SITL/`, then build and **verify the tap on the wire** (a receiver printing the stream).
2. `sim/lidar_imu`: UDP truth -> `PointCloud2` (real pattern, per-point time) + `Imu` 200 Hz -> RViz.
3. `common/frames.py` + `Calib`, then the six convention tests of `FRAMES.md` §5 -- T2/T3/T5 need no
   flight (restart the sim with a different spawn heading).
4. Build Super-LIO and FAST-LIO, feed both the same simulated sensor, bake off.
5. `bridge/` (both implementations) + `fsm/` -> mission A with the acceptance metrics.
6. `planner/` (ego-planner) -> mission B.

## Open questions that block something

See `PLAN.md` §7.  The two that matter soonest: the truth tap's frame (SITL-native NED/FRD, my
preference), and whether the planner chain expects `map` or `odom` as the LIO world's ROS frame name.

## Machine state

* WSL: `/home/fanxin/lio_avoid` is the runtime tree (sim/tools/env.sh mirrored from the repo); the
  ArduPilot clone is `src/ardupilot` and is NOT yet built.
* No simulation is running; nothing is left running from the previous rounds.
* `darklio_*`, `dr_lio_ws`, `drlio_*` and every other project in `/home/fanxin` were left untouched.
