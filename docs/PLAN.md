# PLAN — the contract: goal, decisions, acceptance, order, rules

Living document 1 of 4 (see the documentation budget in §6).  `ARCHITECTURE.md` says how the pieces fit,
`FRAMES.md` says which frame everything is in, `STATUS.md` says where we are.

## 1. Goal

1. **A — fly well**: in the pillar forest, fly the 2.0 x 1.2 m square in GUIDED with the **LIO as the
   EKF's only source** (no GPS, no compass).  Pass = the square is flown accurately *and* the LIO is
   accurate as seen by the FCU.
2. **B — avoid well**: deploy ego-planner and traverse the forest, avoiding pillars by itself.

Every result reports **how many of how many** flights completed and diverged.

## 2. Decisions on record

| # | decision | basis |
|---|---|---|
| 1 | **ArduPilot SITL, built-in quad model**, Copter-4.7.1 pinned | user's choice; no external physics engine |
| 2 | **No Gazebo.**  RViz displays and edits the world; the ray casting is ours | RViz cannot ray-cast; the world is cylinders + planes and the closed-form intersection is exact |
| 3 | **Simulator-only truth tap** in SITL (`libraries/SITL/`, ~30 lines, env-gated) | the lidar needs the true pose at mm precision and hundreds of Hz; SIMSTATE alone is quantised to ~1.1 cm |
| 4 | **Bridge is a switch** (`BRIDGE=mavlink\|mavros`), bidirectional, normalised frame | user wants both; each implementation crosses the frame boundary exactly once |
| 5 | **LIO chosen by measurement** (Super-LIO vs FAST-LIO, same world and mission, N flights each) | both cloned clean; the old Super-LIO verdict carried a known 2.5 deg take-off defect plus harness noise |
| 6 | **Mount: 45 deg nose-down** | measured: at 0.5 m the fan returns 23.5 % floor + 16.6 % ceiling; level would return essentially no floor, and mission A takes its height from the LIO |
| 7 | **World is ENU**, origin at the pad, 10x10 m arena, 46 pillars r = 0.30 m, 0.8 m corridors, the six cells x[0,3] y[0,2] free of cylinders, square centred in that rectangle | user's specification, confirmed by the generator's audit |
| 8 | **LIO input is a standard `PointCloud2`** with fields x,y,z,intensity,time and `lidar_type: 3` | the Livox `CustomMsg` path in this fork **drops the per-point time**, i.e. no motion compensation; the standard path keeps it |
| 9 | **Real MID-360 scan pattern** (`scan_mode/mid360.csv`, 200 kHz, non-repetitive) | measured: azimuth 0-360 deg, elevation -7.16..+52.16 deg, 5 us per point |
| 10 | **GPU ray casting parked**, CPU only | the GPU port failed its own correctness check (2.98 m disagreement); CPU does 20 000 rays in 9.8 ms |

## 3. Acceptance numbers

| line | target | how measured |
|---|---|---|
| A1 route horizontal rms | <= 20 mm | truth vs delivered odometry, route window |
| A2 route vertical rms | <= 50 mm | same window, mean removed |
| A3 per-sample jitter | fair ratio East <= 2x, North <= 5x, Up <= 10x **and** absolute <= 0.50 / 0.50 / 0.15 mm per sample | 2nd difference, route window |
| A4 post-take-off horizontal rms | <= 50 mm | first hold after take-off |
| B1 truth box extents | East <= 2.10 m, North <= 1.26 m | truth only |
| B2 straightness per leg | <= 50 mm | truth only |
| B3 corner arrival | <= 100 mm | distance to the commanded corner |
| B4 landing | LAND + disarm, no failsafe | BIN modes + statustext |
| rates | launched / took off / completed / diverged, for every line | -- |
| **new: LIO as the only EKF source** | `XKF1` vs truth horizontal rms, and "arms into GUIDED with no GPS" | BIN + truth tap |
| **new: B (planner)** | N = 10 traversals, 0 collisions, min clearance >= 0.30 m, goal reached 10/10 | truth + planner status |

Prior evidence that these are achievable: with GPS as the source the accepted configuration flew 10 of
11 flights to completion, 0 diverged, A1 4.4-5.9 mm, A2 31-37 mm (a 2.5 deg frame tilt; 1.7-2.2 mm once
one rotation is allowed), B2 <= 30 mm, B3 <= 68 mm (`docs/figures/round7/`).

## 4. Work order

1. (done) inventory, keep-list, delete the old trees, fresh skeleton
2. (done) analytic ray casting + world generator + real MID-360 pattern + the 0.5 m scan check
3. **the SITL truth tap** -> build Copter-4.7.1 (`modules/mavlink` still missing) -> verify the tap on
   the wire
4. `sim/lidar_imu` -> cloud + IMU -> RViz shows cloud and pillars
5. **`common/frames.py` + `Calib`** and the six convention tests (three need no flight)
6. Super-LIO and FAST-LIO built and fed the same simulated sensor -> LIO bake-off
7. `bridge/` + `fsm/` -> mission A with the acceptance metrics
8. `planner/` -> mission B

## 5. Rules (carried over from the rounds that produced the numbers)

1. Every number carries its frame, origin and window; angles are unwrapped before differencing.
2. No compensating patches: fix the physical cause, and prove it with a controlled A/B.
3. A parameter change must be verified to have reached the consumer.
4. Batch runs quantify rates; one flight is never a conclusion.
5. Every test must also be run against a deliberately wrong configuration and must fail.
6. All WSL work goes through script files (inline `bash -c` gets mangled by the Windows shell).
7. Pin and print revisions: ArduPilot tag, LIO commit, liblio.so md5, world seed.

## 6. Documentation budget (a hard rule from the user, 2026-10-13)

**At most 5 markdown documents in `docs/`, ever.**  Currently 4:

1. `PLAN.md` — this file: goal, decisions, acceptance, order, rules
2. `ARCHITECTURE.md` — components, dataflow, topics, TF tree, transport, safety gates
3. `FRAMES.md` — coordinate conventions, the wire facts, where conversions live, the test matrix
4. `STATUS.md` — current state, evidence, next steps

The fifth slot stays free.  A new subject does **not** get a new file: it goes into whichever of the four
owns it, and if a fifth file is ever created, another must be merged away in the same change.

The count is over the **repo tree** (`D:\distill_self\lio_avoid\docs`), which is where documents live;
the WSL tree (`~/lio_avoid`) is runtime only and carries no markdown.  Run the check before every
commit -- it exits non-zero above 5, or if a document appears that is not registered by name:

```
cd /mnt/d/distill_self/lio_avoid && python3 tools/docs-budget.py
```

Figures, CSVs and the metrics dumps under `docs/` are evidence, not documents, and are not counted (the
budget is about prose that has to be kept true).

## 7. Open questions

1. Truth tap frame: SITL-native NED/FRD (my preference -- one conversion, in one module) or ENU/FLU?
2. Does the planner chain (ego-planner, RViz configs) assume a particular fixed frame or TF shape?  That
   decides whether the LIO's world is published as `odom` (current plan) or as `map`.
3. Should the simulator still carry a compass (never an EKF source) so heading can be cross-checked?
4. The old `gazebo_apm` tree holds ~40 historical documents (the evidence chain for every number
   reported in rounds 1-7).  It is a frozen archive rather than a living document set: fold it into the
   budget as well, or leave it frozen?
