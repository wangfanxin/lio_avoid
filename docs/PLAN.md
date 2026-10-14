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
| 11 | **Rectangle first; ego-planner deferred, its interface built now** (`/planner/traj`, `nav_msgs/Path`, frame L, §4.1) | user's call: prove the whole chain on the square before adding a planner.  Because the rectangle source publishes the *same* interface, mission A also validates the integration point, and adding the planner later cannot change the FSM |
| 12 | **Setpoints and telemetry at 50 Hz** (`SR1_POSITION = 50`, `SR1_EXTRA1 = 50`; vision 50 Hz) | 50 Hz is the firmware's own declared ceiling for the stream-rate parameters, and the bridge verifies the achieved rates with its own counters (`ARCHITECTURE.md` §4b) |
| 13 | **The truth tap speaks SITL-native NED/FRD** | user's call on my recommendation (2026-10-13): the wire format is the simulator's own, and every conversion then lives in one place (`common/frames.py`) |
| 14 | **The vehicle spawns facing North** | user's call (2026-10-13), `FRAMES.md` §0.3 option 1: FCU local NED is the textbook `(N, E, -U)` permutation with no yaw anywhere, L = W rotated -90 deg.  The spawn condition is **asserted from the truth tap before every take-off**, never assumed |
| 15 | **Mission A flies with `EK3_SRC1_POSZ = 1` / `VELZ = 0` first; the LIO's z follows immediately after** | user's call (2026-10-13): "先 a 后 b，最后一定得是 b 通过的" -- unblock the milestone on the barometer, then fix the 2.5 deg tilt so that `POSZ = 6` passes too.  **Phase A is only closed when (b) passes**: the tilt is not allowed to become permanent |

## 3. Acceptance numbers

| line | target | how measured |
|---|---|---|
| A1 route horizontal rms | <= 20 mm | truth vs delivered odometry, route window |
| A2 route vertical rms | <= 50 mm | same window, mean removed |
| A3 delivered-odometry continuity | delivery: rate >= 80% of the LIO's 200 Hz and no gap > 3x the median period; wander: rms 2nd difference <= 2.0 mm per axis (10% of A1); outliers: p99 single step <= 10.0 mm | the samples as DELIVERED (no resampling: the FCU never interpolates), route window |
| A4 post-take-off rms | <= 50 mm horizontal, mean removed | the FCU's own `XKF1` estimate vs the truth, its clock aligned to the sim clock by the take-off ramp (lag reported) |
| B1 truth box extents | East <= 2.10 m, North <= 1.30 m (command 2.000 / 1.200, so +0.10 on both axes, the same tolerance as B3); the legacy N <= 1.26 is still counted beside it | truth only, route window |
| B2 straightness per leg | <= 50 mm | truth only |
| B3 corner arrival | <= 100 mm | distance to the commanded corner |
| B4 landing | LAND + disarm, no failsafe | BIN modes + statustext |
| rates | launched / took off / completed / diverged, for every line | -- |
| **new: LIO as the only EKF source** | `XKF1` vs truth horizontal rms, and "arms into GUIDED with no GPS" | BIN + truth tap |
| **new: B (planner)** | N = 10 traversals, 0 collisions, min clearance >= 0.30 m, goal reached 10/10 | truth + planner status |

Prior evidence that these are achievable: with GPS as the source the accepted configuration flew 10 of
11 flights to completion, 0 diverged, A1 4.4-5.9 mm, A2 31-37 mm (a 2.5 deg frame tilt; 1.7-2.2 mm once
one rotation is allowed), B2 <= 30 mm, B3 <= 68 mm (`docs/figures/round7/`).

## 4. Work order (reordered 2026-10-13: rectangle first, the planner DEFERRED but its interface built now)

1. (done) inventory, keep-list, delete the old trees, fresh skeleton
2. (done) analytic ray casting + world generator + real MID-360 pattern + the 0.5 m scan check
3. **the SITL truth tap** -> build Copter-4.7.1 (`modules/mavlink` still missing) -> verify the tap on
   the wire
4. `sim/lidar_imu` -> cloud + IMU -> RViz shows cloud and pillars
5. **`common/frames.py` + `Calib`** and the six convention tests (three need no flight)
6. the LIO: build the clean upstream, carry over the measured configuration, feed it the simulated
   sensor, verify its odometry against the truth tap.  (The Super-LIO vs FAST-LIO bake-off is a later
   refinement, not a gate: one LIO working end to end is what mission A needs.)
7. **MISSION A -- the rectangle.**  `planner/rectangle_source` publishes the trajectory interface of
   §4.1; `bridge/` + `fsm/` consume it.  This is the milestone: the square is flown with the LIO as the
   EKF's only source, and A1-A4 / B1-B4 are measured with their counts.
8. **DEFERRED -- ego-planner.**  Nothing in the FSM changes: a ~30-line adapter turns the planner's
   B-spline into the same `/planner/traj`, and then mission B is flown through the identical path.

### 4.1 The trajectory interface (built now, used by the rectangle, filled by the planner later)

| item | value |
|---|---|
| topic | **`/planner/traj`** |
| type | **`nav_msgs/Path`** (poses with stamps), frame **L** |
| rate | points at >= 20 Hz of trajectory time; republished at >= 2 Hz so a consumer that joins late still has a fresh horizon |
| content | position + yaw per point; dense enough that the FSM can difference it for velocity and acceleration |
| producer today | `planner/rectangle_source` -- the square from `worlds/pillars.yaml`, corner holds included |
| producer later | `planner/ego_adapter` -- samples the planner's B-spline into the same Path; the planner itself is untouched |
| FSM's obligations | sample it at 50 Hz, convert L -> F, stream the setpoints, enforce the safety gates, publish `/fsm/phase` |
| source's obligations | keep the path inside the arena, leave >= **0.20 m** (`--footprint`) to any pillar's TRUE surface in the world file, and never publish a path that starts further than 0.5 m from the current state |

**The clearance contract is 0.20 m** (decided by the user 2026-10-14: *"我希望这个值是0.2m"*; it was 0.35 m).
It is measured by `planner/ego_adapter.World.clearance()` against the world file's analytic cylinders --
not against the planner's inflated occupancy grid -- so it is independent of `grid_map/obstacles_inflation`,
which the user set to **0.10 m** the same day (from 0.45 m).  The two numbers do different jobs: 0.20 m is
what the project promises, 0.10 m is what the planner believes, and the promise is the binding one.  The
consequence for any world is then a formula: a corridor of width `g` is flyable while `g > 2 x 0.20 =
0.40 m`, and visible to the planner at all while `g > 2 x 0.10 = 0.20 m`.  The 1.5 m / r = 0.4 m forest of
`worlds/forest_b.yaml` has `g = 0.565-0.640 m`, so it is both flyable AND threadable -- which is what makes
mission B a real avoidance problem rather than a corridor-following exercise.

Why a `nav_msgs/Path` and not a bespoke message: it carries what both sources can produce without a new
package, and it keeps the *planner's* message types out of the flight-critical code -- the adapter is the
only file that will ever include `traj_utils`.  If the planner turns out to need acceleration feed-forward
that a Path cannot express, the interface can be extended then, with the rectangle source proving the old
form still works.

### 4.2 PHASE A in detail — truth tap to a flown rectangle (for review, 2026-10-13)

Each step is: **build → verify with a named gate → risks and the fallback if the gate fails.** Steps are
sequential, and every gate prints a number: a step is not finished because code exists.

**S3. The SITL truth tap — DONE 2026-10-13, all three gates pass**
*Build*: `git submodule update --init --depth 1 modules/mavlink` (currently empty), ArduPilot's own python
requirements, `./waf configure --board sitl` + `./waf copter`; then ~30 lines in `libraries/SITL/`, gated by
an environment variable, sending one line per model step (t, p_ned, v_ned, q body→NED, body rates, specific
force) to a UDP port. `sim/truth_rx.py` receives and republishes `/clock`, `/sim/rtf`, `/sim/truth/odom`.
*Gate*: a parked 30 s run — tap rate ≥ 380 Hz, `|a|` within 0.05 of 9.7946, quaternion norm 1 ± 1e-6,
position drift < 1 mm, and the tap's position agreeing with SITL's own `SIM2` log to < 1 mm. Plus a scope
check: `git diff` shows only the tap file(s) — the flight code is untouched.
*Risks*: submodule fetch (fallback: the gitee mirror, or fetch only the mavlink headers); build deps
(`empy`, `pexpect`, `lxml`, `pyparsing`); hooking the tap somewhere that disturbs timing (it must not).

*Result*: 67 added lines in one file, delivered as `patches/0001-sitl-truth-tap.patch` and re-applied by
`tools/apm-apply-patch.sh` (the re-applied diff is md5-identical, so the patch is the change).  Parked
30 s: **1200.5 Hz**, `|a|` 9.8067, `a_z` negative, quaternion norm deviation 0, worst gap 0.8 ms,
position span 0.000000 m, 0 UDP drops; against the firmware's own SIM2 record: **position max 0.0004 mm,
clock bias +0.0000 ms, drift +0.06 ppm**.  Three facts the rest of the phase inherits: the model steps at
1200 Hz (not 400); `--serial0 tcp:5760` must be passed WITHOUT `:wait` or the sketch never starts; parked
BINs need `LOG_DISARMED 1`.  Evidence: `docs`-adjacent run dirs `logs/s3-parked-*/`, `logs/s3-ros-*/`
(see `STATUS.md`).

**S4. The sensor node and RViz**
*Build*: `sim/lidar_imu.py` (truth + world → `PointCloud2` in S at 10 Hz with real per-point times, `Imu`
in S at 200 Hz), `sim/world_markers.py`, `rviz/*.rviz`.
*Gate*: achieved rates from the node's own counters (cloud 10.0 ± 0.2 Hz, IMU 200 ± 2 Hz); per-point time
monotonic over 0…0.099995 s; and the strong one — **accumulate 60 s of cloud, fit the pillar centres, compare
with `worlds/pillars.yaml`**: agreement to a few mm validates the mount, the pattern, the timing and the
frame chain *before any LIO exists*. RViz: with Fixed Frame `mid360_link` the raw scan is a clean fan; with
`world_w` the pillars and the accumulated cloud coincide.
*Risks*: none serious (9.8 ms/scan measured); if the rates drift, `/sim/rtf` says so.

**S5. `common/frames.py`, `Calib`, and the convention tests**
*Build*: `frames.py` (the four conversions), `Calib` (measure, then compare against the construction
prediction), `tools/conv-test-*.py` for T1, T2, T3, T3b, T5 of `FRAMES.md` §5.
*Gate*: T2/T3/T3b/T5 run **parked, unarmed** (restart the sim per spawn heading); T1 needs one short guided
move along each axis. Each test prints PASS/FAIL against an explicit expected value, and **each has a
mutation case**: flip a sign in `frames.py` and the test must fail. A test that passes with the sign flipped
has no power and is not a gate.
*Risks*: the spawn heading must be asserted from the tap, and if SITL starts the vehicle somewhere else that
is exactly what T2 measures — it gets fixed in the simulator, never papered over in the FSM.

**S6. One LIO, end to end**
*Build*: vendored `livox_ros_driver2` (so Super-LIO's CMake stays pristine — it requires that package
unconditionally), clean upstream Super-LIO built with colcon, and `lio/config/livox_360.yaml` carrying the
**measured** values from the old tree, adapted to our input: `lidar_type: 3`, `blind 0.3`, `maxrange 40`,
`ds_size 0.15`, `voxel_fliter_size 0.2`, `filter_rate 1`, `imu_ng 0.005`, `imu_na 0.1`,
`gravity_norm 9.7946`, `lidar_imu` identity, `odom_robo` = 45° mount + (0.05, 0, 0.10); outputs `/odom`
(fitted, 200 Hz), `/cloud_registered`, `/lio/status`.
*Result*: the clean upstream (`f89f48d`, `liblio.so` md5 `e4b82a73cbfb70a7a538514f74936b09`) built with
the vendored driver and fed by our sensor publishes `/lio/odom` at **10.02 Hz** and `/lio/robo/odom` at
**199.26 Hz**; parked 40 s the drift is **26.60 mm max / 9.43 mm rms**.  The one thing that made those
numbers mean anything was turning the sensor's noise ON: with the exact truth as the IMU the same LIO
tracked to **0.01 mm**, which is a measurement of a simulation with no noise in it, not of an estimator.
Motion and the tilt (gate (b)/(c)) move to S7 -- see `STATUS.md` row 28 for why the shove harness cannot
answer them.
*Risks*: CPU/RTF (LIO ≈ 1 core; gate `/sim/rtf` ≥ 0.8); the LIO's world-yaw convention (the parked test);
the tilt — see the branch below.

**S7. Mission A — the rectangle**
*Build*: `planner/rectangle_source` (§4.1), `bridge/` (both implementations), `fsm/` (sampler, arrival
checks, safety gates, `/fsm/phase`), `tools/recorder` + `tools/metrics` (ported: A1–A4, B1–B4, and counts).
*Procedure*: pre-flight assertions (S5 green, `Calib` matches construction, EKF origin set without GPS,
arming checks as configured) → **N = 10 flights**, each recording everything and reporting `/sim/rtf` and the
achieved rates.
*Gate*: §3 numbers, every line with "how many of how many", 0 diverged. A flight aborted for a harness reason
is counted and explained, never dropped silently.
*Risks and fallbacks*: no-GPS arming (the old `ARMING_SKIPCHK`/`VISO_*` block is the starting point);
`mavros` vs `pymavlink` differences (both are gates, not alternatives); RTF collapse (the old project's
aborts) → an RTF gate before every flight; below 0.8 the flight is not started.

**The one branch that can stop Phase A**

`EK3_SRC1_POSZ = 6` makes the altitude loop eat the LIO's z, and a pitch error costs `theta × distance` in
height: at the 2.5° the old LIO showed that is **43 mm per metre**, i.e. ~86 mm over the 2 m leg against an
A2 target of 50 mm — A2 would fail even though the estimator is fine.

| option | what it means | cost |
|---|---|---|
| **(a) fly A with `EK3_SRC1_POSZ = 1` (baro) and `VELZ = 0`, keeping position and yaw on the LIO** | SITL has a barometer, so this works today; A2 then measures the altitude loop, not the LIO's z. It is the old project's own rectangle configuration, so the path is proven — but it dodges the question | A is not blocked; the LIO's z stays unproven |
| (b) fix the tilt first (instrument the LIO's state through the climb, find where the 2.5° enters) | A is flown with all five sources on the LIO, as originally intended | delays A by an unknown amount (C++ instrumentation + rebuild cycles) |
| (c) fly A with `POSZ = 6` anyway | measures the real thing | A2 fails by construction; not a plan |

Recommendation: **(a) to unblock A, (b) immediately after A**, and the report says explicitly that A2's
number under (a) is the barometer's, not the LIO's.

**DECIDED (user, 2026-10-13): (a) then (b), and Phase A is only closed when (b) passes** -- see §2 #15.
So S7 is flown twice: once with `POSZ = 1` to get the milestone and the horizontal numbers, and once with
`POSZ = 6` after the tilt is fixed, which is the run that closes the phase.

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
5. **DECIDED 2026-10-13 -- spawn heading: North** (§2 #14).  The tap frame is DECIDED too: SITL-native
   NED/FRD (§2 #13).
6. **DECIDED 2026-10-13 -- the 2.5 deg tilt: option (a) first, then (b)** (§2 #15).  The milestone is flown
   with the barometer as the altitude source, and Phase A does not close until `POSZ = 6` passes as well.
