# PLAN — the contract: goal, decisions, acceptance, order, rules

Living document 1 of 4 (budget in §7).  `ARCHITECTURE.md` says how the pieces fit, `FRAMES.md` says which
frame everything is in, `STATUS.md` says where we are.

**THE PROJECT HAS ONE MISSION.**  On 2026-10-15 the owner dropped the ego-planner line (mission B).  Mission
A — the rectangle, flown by `src/planner/rectangle_source.py` + `fsm/mission_fsm.py`, with **no planner
process in the loop** — is the working pipeline, and it is what every number in these documents is about.
What mission B taught us is kept in `STATUS.md`, in one section at the end.

## 1. Goal

**A — fly well**: in the pillar forest, fly the 2.0 x 1.2 m square in GUIDED with the **LIO as the EKF's
only source** (no GPS, no compass).  Pass = the square is flown accurately *and* the LIO is accurate as
seen by the FCU, *and* the aircraft never comes inside the footprint contract of a pillar, *and* it lands.

Every result reports **how many of how many** flights completed and diverged.

The owner's later requirement, which Mission A now carries as well: **hover at every corner**
(`RECT_HOLD_S=5`).  **DEMONSTRATED 2026-10-15** on `logs/missionA-RECT7-hold5`: `src/tools/dwell-audit.py`
prints **4 of 4 corners hovered >= 5.0 s**, and 4 of 4 again with the stricter airborne-only measurement
(`STATUS.md` row 43).  **RE-DEMONSTRATED 2026-10-16 on the refactored architecture** (`STATUS.md` row 52),
where the dwell is no longer a convention between two processes but content of ONE command stream: the PLAN
box commands `position = the corner, velocity = 0` for `--hold-s`, after the vehicle has ARRIVED inside
`--arrive-m` (`STATUS.md` row 55).

## 2. Decisions on record

| # | decision | basis |
|---|---|---|
| 1 | **ArduPilot SITL, built-in quad model**, Copter-4.7.1 pinned | user's choice; no external physics engine |
| 2 | **No Gazebo.**  RViz displays and edits the world; the ray casting is ours | RViz cannot ray-cast; the world is cylinders + planes and the closed-form intersection is exact |
| 3 | **Simulator-only truth tap** in SITL (`libraries/SITL/`, 67 lines, env-gated) | the lidar needs the true pose at mm precision and hundreds of Hz; SIMSTATE alone is quantised to ~1.1 cm.  **The tap stays for mission A: C2, A1–A4 and B1–B4 are all computed from it** |
| 4 | **Bridge is a switch** (`BRIDGE=mavlink\|mavros`), bidirectional, normalised frame | user wants both; each implementation crosses the frame boundary exactly once |
| 5 | **LIO chosen by measurement** (Super-LIO vs FAST-LIO, same world and mission, N flights each) | both cloned clean; the old Super-LIO verdict carried a known 2.5 deg take-off defect plus harness noise.  The bake-off is a refinement, not a gate: one LIO working end to end is what mission A needs |
| 6 | **Mount: 45 deg nose-down** | measured: at 0.5 m the fan returns 23.5 % floor + 16.6 % ceiling; a level mount would return essentially no floor, and mission A takes its height from the LIO |
| 7 | **World is ENU**, origin at the pad, 46 pillars r = 0.30 m, 0.8 m corridors, the six cells x[0,3] y[0,2] free of cylinders, square x[0.5,2.5] y[0.4,1.6] at 2.0 m inside that rectangle | user's specification, confirmed by the generator's audit; worst pillar clearance on the square **0.600 m** |
| 8 | **LIO input is a standard `PointCloud2`** with fields x,y,z,intensity,time and `lidar_type: 3` | the Livox `CustomMsg` path in this fork **drops the per-point time**, i.e. no motion compensation; the standard path keeps it |
| 9 | **Real MID-360 scan pattern** (`scan_mode/mid360.csv`, 200 kHz, non-repetitive) | measured: azimuth 0-360 deg, elevation -7.16..+52.16 deg, 5 us per point |
| 10 | **GPU ray casting parked**, CPU only | the GPU port failed its own correctness check (2.98 m disagreement); CPU does 20 000 rays in 9.8 ms |
| 11 | **Rectangle first** (`/plan/cmd`, `quadrotor_msgs/PositionCommand`, frame L, §4.1) | user's call: prove the whole chain on the square before adding a planner.  **Closed 2026-10-15**: a planner was built behind that interface and flown, then DROPPED by the owner; the interface itself is what made the drop a deletion instead of a rewrite (not one line of the FSM changed).  **AMENDED 2026-10-16 (F1)**: the interface became the COMMAND STREAM the design had specified (`_stage/refactor-bridge-plan.md` §3.3) -- a `nav_msgs/Path` says where to go but not what to command, so it needs an executor on the other side, and that executor is the box a whole-replacement swaps out.  The Path and the FSM's corner walk are DELETED (`STATUS.md` row 52) |
| 12 | **Setpoints and telemetry at 50 Hz** (`SR1_POSITION = 50`, `SR1_EXTRA1 = 50`; vision 50 Hz) | 50 Hz is the firmware's own declared ceiling for the stream-rate parameters, and the bridge verifies the achieved rates with its own counters (`ARCHITECTURE.md` §6) |
| 13 | **The truth tap speaks SITL-native NED/FRD** | user's call on my recommendation (2026-10-13): the wire format is the simulator's own, and every conversion then lives in one place (`src/common/frames.py`) |
| 14 | **The vehicle spawns facing North** | user's call (2026-10-13), `FRAMES.md` §2: the FCU local NED is the textbook `(N, E, -U)` permutation with no yaw anywhere, and L = W rotated -90 deg.  The spawn condition is **asserted from the truth tap before every take-off**, never assumed |
| 15 | **Mission A flies with `EK3_SRC1_POSZ = 1` / `VELZ = 0` first; the LIO's z follows immediately after** | user's call (2026-10-13): "先 a 后 b，最后一定得是 b 通过的".  **Both are now flown**: `params/phase_a_posz_baro.parm` is step (a), `params/phase_a_posz_lio.parm` — `POSZ = VELZ = POSXY = YAW = 6` — is step (b), and every flight in `docs/metrics/stepB-rectangle-20261013.txt` is step (b) |
| 16 | **The frozen mission-A set** (49 files, `frozen/mission_a.manifest`) | user's instruction *"mission A之前的任务和rviz的地图都保存好，冻结，不搞了"*.  The five worlds it was flown in, its parameter sets, `rviz/mission_a.rviz`, its figures and the metric dumps that carry its numbers are byte-frozen; `src/tools/frozen-check.py` recomputes every SHA-256 and must print `49 of 49 files unchanged` before every commit |
| 17 | **The clearance contract is 0.20 m** | user's decision 2026-10-14 (*"我希望这个值是0.2m"*), down from 0.35 m.  Measured by `src/tools/metrics.py` C2 on the tap's truth, against the world file's analytic cylinders.  **AMENDED 2026-10-16 (owner's decision)**: the 0.20 m contract stands unchanged for the stack's own arm, and the EGO arm is now judged by its own row, **C2-ego** -- *>= 95 % of truth samples at >= 0.10 m from every pillar surface, and 0 samples inside a pillar body*.  **0.10 m is not a rounded-down 0.20 m: it is ONE MAP CELL, and at this field's tightest pair it IS the free channel ego's own map leaves** (0.814 m face to face, `obstacles_inflation` 0.099 m at 0.1 m resolution = one cell, so `inf_step = 1`, and the inflation is a 27-cell **cube**: 0.616 m of channel on the axes, 0.532 m at 45 deg where the cube's corner reaches 0.141 m).  Reason it moved: **0 of 32 ego crossings meet 0.20 m**, and a row no crossing can meet measures nothing -- while the restated row discriminates (23 of 32 for the fraction, 10 of 32 for the body count).  Recorded as a RELAXATION, with what it does *not* claim, in `docs/metrics/c2-prime-channel-20261016.txt` |
| 18 | **The flight configuration is NAMED, not inherited from three knobs' separate defaults** | The mission-B review (P2-4) found the defaults contradicting the measured record: `EGO_CLOUD_WINDOW` defaults to 0.0 while rows 73-75 say the map's memory is necessary, `EGO_SELF_BLIND` 0.0 while the launch comment calls it the enabler for two-cell inflation, and the "current best known" arm exists only as three unrelated values.  **Decision**: the conservative baseline stays the default -- so that every historical row keeps meaning what it says, and so Flight A tests ONE correction rather than four -- and the best-known arm is PINNED as a named preset (`_stage/best-known-config.sh`, with its evidence and its caveats printed beside it: relay window 8.0 s measured on the ADAPTER sampler, `dist0` 0.5, one cell of inflation, `EGO_CONTRACT=shim`, `BRIDGE_FIELDS=posvel`).  A flight records which of the two it flew. |
| 19 | **The three "task finished" rules are a TABLE, and the shell prints them at start-up** | Review P3-2: ego decides arrival itself (`plan_done && wp_d_veh < fsm/waypoint_advance_m`, 1.000 m by launch default), the contract shell decides it again (`--arrive-m` 0.30 m, `|v| < --settle-mps` 0.08 m/s, held `--hold-s` 5.0 s), and the supervisor decides it a third time (`trajectory_flag = COMPLETED`; its 3.0 s plan-silence window is a FAILURE rule, not an arrival rule).  **Decision**: keep all three -- they answer different questions (ego: when to plan the next leg; the shell: what to put on the contract; the supervisor: whether the mission ended) -- but make them non-discoverable-by-archaeology: `ego_contract_shim.py` prints all of them with their sources on every start-up, and `src/planner/ego_source.py --selftest` keeps the shell's own rule asserted. |

## 3. Acceptance numbers

All measured by `src/tools/metrics.py` from one run's recordings (`tap.csv`, `lio.csv`, `phase.csv`, the FCU's
BIN), offline and after the flight.  Every line prints its denominator.

| line | target | how measured |
|---|---|---|
| A1 route horizontal rms | <= 20 mm | truth vs delivered odometry, route window |
| A2 route vertical rms | <= 50 mm | same window, mean removed |
| A3 delivered-odometry continuity | delivery: rate >= 80 % of the LIO's 200 Hz and no gap > 3x the median period; wander: rms 2nd difference <= 2.0 mm per axis (10 % of A1) **over the route window**, with the whole-stream value printed beside it as reported-not-gated; outliers: p99 single step <= 10.0 mm | the samples as DELIVERED (the FCU never interpolates), route window for the wander (take-off and landing are where a second difference measures the climb, not the estimator), every sample for the delivery and outlier gates |
| A4 post-take-off rms | <= 50 mm horizontal, mean removed | the FCU's own `XKF1` estimate vs the truth, its clock aligned to the sim clock by the take-off ramp (lag reported) |
| B1 truth box extents | East <= 2.10 m, North <= 1.30 m (command 2.000 / 1.200, so +0.10 on both axes, the same tolerance B3 uses); the legacy `N <= 1.26` is still counted beside it | truth only, route window |
| B2 straightness per leg | <= 50 mm | truth vs the line through that leg's own endpoints |
| B3 corner arrival | <= 100 mm | truth to the commanded corner |
| B4 landing | LAND + disarm, no failsafe | the phase log + the BIN's modes and statustext |
| C1 final point | reported, not gated | distance from the last commanded point when the mission ends |
| **C2 obstacle clearance** (stack arm: rectangle / FSM) | **0 samples inside the 0.20 m footprint** | every truth sample above 0.30 m against the world file's 46 cylinders |
| **C2-ego** (ego arm) | **>= 95 % of samples at >= 0.10 m from every pillar surface, and 0 samples inside a pillar body** | same truth and same analytic cylinders, from `_stage/c2-prime.py`; 0.10 m is one map cell and equals the free channel at this field's tightest pair (decision 17).  Historical record under this row: **23 of 32** ego crossings for the fraction, **10 of 32** for the body count -- against **0 of 32** for the old 0.20 m.  The map-frame form of the same sentence ("never in a cell ego's map marks OCCUPIED") is a DIAGNOSTIC, not the bar: an unmarked pillar is a free cell, so a blind map passes it |
| rates | launched / took off / completed / diverged, for every line | `src/tools/run-mission-batch.sh` + `src/tools/batch-summary.py` |

Prior evidence that these are achievable: with GPS as the source the accepted configuration flew 10 of 11
flights to completion, 0 diverged, A1 4.4-5.9 mm, A2 31-37 mm (a 2.5 deg frame tilt; 1.7-2.2 mm once one
rotation is allowed), B2 <= 30 mm, B3 <= 68 mm (`docs/figures/round7/`).  On the LIO as the only source,
step (b): **A1 10 of 10 (4.650-4.900 mm), A2 10 of 10 (4.560-4.890), A3 10 of 10, A4 10 of 10
(18.640-21.090), B1 10 of 10, B2 39 of 50 legs, B3 40 of 40 corners, B4 10 of 10, completed 10 of 10**
(`docs/metrics/stepB-rectangle-20261013.txt`).  B2 is the one open acceptance gap.

### 3.1 Mission B on `worlds/rect68.yaml` — the arm, the frontier, and which clause is met

The owner's objective for mission B is **every waypoint leg <= 6 s AND no collision** (the truth never inside a
pillar body, minimum surface distance >= 0).  As of 2026-10-17 the arm and the numbers are:

| knob | value | why |
|---|---|---|
| `EGO_CORRIDOR_M` | **0.01 m** (`END_M`/`OPEN_M` 0.0) | holds the forwarded reference on the leg centreline, which is the field's maximum-clearance path: every edge is exactly +0.200 m from the nearest pillar surface while the plan snaked +-0.4-0.5 m about it.  Took the collision rate from 7.42 % to 0.06 % of samples (STATUS 188-198) |
| `EGO_CORNER_M` / `MPS` | **0.50 m / 1.00 m/s** | the corner is where the aircraft's momentum is made; the governor's radius was 1.0 m and 8 corners is ~16 m of a 44 m route, so it set the schedule (|v_ref| p50 == the cap) |
| `EGO_MAX_VEL` / `ACC` | **2.5 m/s** | the clean band.  2.75 and 3.0 each bought seconds at the cost of a negative minimum (STATUS 192-198) |
| the rest | Flight AD's | posvelacc, ref-clamp 1.2, ref-slew 6.0, ref-acc 2.0, ref-z 0.6/0.6, settle 0.0, dwell 0.0, `PSC_NE_POS_P` 1.5, 10 m pillars, lidar map 0.1 m, PATCH I + RESOLUTION/1 + the crash fix |

A **bare `PLANNER=ego` sortie** flies it, and the last verification (Flight AY, no environment at all) read:
C2 minimum **-0.003 m**, **0.06 %** of samples inside a body, 1.00 % below the 0.10 m standoff, A1 10.32 / A2 10.68 mm,
B1 E 6.295 / N 8.392, C1 0.140 m, route 38.0 s, **6 of 8 legs under 6 s (max 6.77 s)**.

**Both criteria have been met, in different sorties, and the gap between them is one knob**: Flight AT (+0.029 m,
0 of 1898 samples inside, 6 of 8 legs) and AU (**8 of 8 legs at 3.55-5.30 s**, minimum -0.007 m) differ only in
`EGO_MAX_VEL`/`ACC` (2.5 vs 3.0).  The default is the clean side, because "0 samples inside a body" is this table's
hard gate and a leg over 6 s is a performance miss; `EGO_MAX_VEL=3.0 EGO_MAX_ACC=3.0` converts one into the other.
The run-to-run spread on the minimum clearance, measured across the day's 19 sorties, is ~0.04 m at a fixed arm, so
"clean" here means "inside one spread of zero", and that is stated rather than claimed away.

## 4. The pipeline

```
worlds/pillars.yaml ──► src/sim/lidar_imu.py ──► /livox/lidar + /livox/imu ──► Super-LIO
      (one geometry                            (10 Hz, 20 000 pts)         (200 Hz, L)
       source: src/tools/                                │                         │
       world-info.py)                                │                         ▼
                                                     │              /lio/robo/odom (pose, 200 Hz, L)
                                                     │              /lio/odom      (velocity, 10 Hz, L)
                                                     │                         │
   SITL ── truth tap ──► src/sim/truth_rx.py ──► /sim/truth/odom (W)               │
    ▲        (1200 Hz, NED/FRD)                  │                             │
    │                                            │                             ▼
    │                                     (geofence gate only)   src/planner/rectangle_source.py
    │                                                              THE PLAN **AND ITS EXECUTION**:
    │                                                              the square's five waypoints in L,
    │                                                              its OWN arrival gate + settle +
    │                                                              dwell from /lio/robo/odom
    │                                                                           │
    │                                                              /plan/cmd (L, 100 Hz, its own timer)
    │                                                                           │
    │                                                              src/bridge/bridge_node.py (relay, 50 Hz)
    │                                                                           │
    └──── MAVLink 84 / 76 / 102 / 331 ◄──────────── fsm/mission_fsm.py ─────────┘
                     src/bridge/mavlink_impl.py          (preflight, takeoff, hold, the `leg` phases
                                                      read off the stream, geofence, LAND, abort)
```

The one edge not drawn: the PLAN box **reads** `/lio/robo/odom` — the same LIO pose, converted L -> W with
the same `--calib-file` it publishes with — to decide when a leg has been flown.  That is the arrival gate,
the settle and the dwell (§4.1), and it is the whole reason the plan box is an executor rather than a
publisher.  It never reads the truth tap: a planner swapped in here has no truth tap to read.

Nothing plans.  The trajectory is the world file's square, published as a command stream; the plan box's own
sequencer visits its waypoints and holds each one; ArduPilot GUIDED flies the setpoints; the truth tap judges
the result.  There is no planner process, no planner workspace and no planner message type anywhere in the
tree.

### 4.1 The trajectory interface (the PLAN box's output — one producer, and it is the rectangle)

**AMENDED 2026-10-16 (F1).**  Until F1 this was a `nav_msgs/Path` on `/planner/traj` and the FSM was the
executor.  A Path says where the vehicle should be but not what to command, so it needs an executor on the
other side — and under the owner's amendment ("EXECUTOR 应该是 PLAN 的一部分") that executor is the box a
whole-replacement swaps out.  The interface is now the command stream ego's own `traj_server` publishes, and
the Path, the FSM's corner walk and `--follow path` are deleted with it.

| item | value |
|---|---|
| topic | **`/plan/cmd`** |
| type | **`quadrotor_msgs/msg/PositionCommand`** (vendored, byte-identical to ego's: `ros_ws/src/quadrotor_msgs`), frame **L** (`header.frame_id = lio_world`; ego's name for the same frame, `world`, is accepted by configuration) |
| rate | published at **100 Hz** on the plan box's own wall timer; **consumed at 50 Hz** — the wire rate stays 50 Hz (decision 12, `ARCHITECTURE.md` §6) |
| fields used | position, velocity, acceleration, yaw, yaw_dot, `trajectory_id`, `trajectory_flag` |
| the leg | `trajectory_id` **increments once per waypoint**, and that is what the consumer turns into the `leg1..leg5` phases the metrics window with (risk R1) |
| the dwell | **content**: position = the corner, velocity = 0, for `--hold-s`, and the clock starts when the vehicle has ARRIVED AND SETTLED at the corner |
| the end of the tour | `trajectory_flag = TRAJECTORY_STATUS_COMPLETED`, and the stream **keeps publishing** the hover afterwards — so "finished" and "stopped talking" are different facts and the consumer can tell them apart (`src/bridge/bridge_node.py::CmdRelay.stalled_reason` names the second) |
| producer | `src/planner/rectangle_source.py` + `src/planner/cmd_stream.py` — the square from `worlds/pillars.yaml`, with the corner hold (`RECT_HOLD_S`, **5.0 by harness default**) |
| producer's obligations | take the geometry from the world file through `src/tools/world-info.py`, convert W -> L with the **same** `--calib-file` the bridge uses, and keep the plan inside the arena |
| consumer's obligations | `src/bridge/bridge_node.py` relays it to message 84 at 50 Hz through the ONE entry point (`bridge.send_cmd(cmd_l, calib)`); `fsm/mission_fsm.py` derives the phase log from it, keeps the truth-fed geofence, and keeps a NAMED abort for silence |

**The corner hold's shape on this interface.**  The plan box commands **one waypoint at a time** out of the
tour `[c1, c2, c3, c4, c1]` (the first waypoint is the ENTRY: the aircraft starts on the pad, and the tour's
dwells belong to the corners of the tour, so the entry leg has none — the same five `leg` phases the FSM's
corner walk produced).  A leg's flight clock arms when the vehicle has left the pad and is within
`--leg-start-m` of the leg's start; the arrival is declared when the vehicle is inside `--arrive-m` **and**
slower than `--settle-speed`, with `--settle-s` as the bound; `--leg-timeout-s` bounds the wait and names its
reason.  Every one of those decisions is made from `/lio/robo/odom` — the plan box's own odometry — and
`src/tools/dwell-audit.py` judges the result from the truth tap, never from either log.

**What the interface deliberately does not carry**: a republish rate a slow planner can meet, a
WITHHOLD-with-a-reason for an expired plan, the re-projection of a plan onto the vehicle, the station
sequence and its observer.  All of it went in the 2026-10-15 cleanup.  The clearance contract is unchanged
for this arm and still enforced where it always was: **C2 measures it on the flight's own truth**, and the
square's own geometry holds **0.600 m** to the nearest pillar by construction.  (The EGO arm got its own row
on 2026-10-16, decision 17: C2-ego, 0.10 m for >= 95 % of samples plus 0 in a pillar body -- the same truth
and the same cylinders, a different number, because 0 of 32 ego crossings meet this arm's 0.20 m.)
The PLAN's own clearance column is
`traj.csv`'s `clr_m`, which F1 re-pointed at this stream in the same change (§7 risk R5).

Why the adopted type and not a bespoke one: it is what ego already publishes and what upstream's own
consumer (`so3_control`) reads, so a planner drops in with a launch remap instead of an adapter — and the
adapter that translated ego's output into a Path is exactly what this change deleted.

## 5. Work order, and where it stands

1. (done) inventory, keep-list, delete the old trees, fresh skeleton
2. (done) analytic ray casting + world generator + real MID-360 pattern + the 0.5 m scan check
3. (done) **the SITL truth tap**, Copter-4.7.1 built, tap verified on the wire
4. (done) `src/sim/lidar_imu` -> cloud + IMU -> RViz shows cloud and pillars
5. (done) **`src/common/frames.py` + `Calib`** and the convention tests
6. (done) the LIO: clean upstream built, fed by the simulated sensor, verified against the tap
7. (done) **MISSION A — the rectangle.**  `src/planner/rectangle_source` publishes the interface of §4.1;
   `src/bridge/` + `fsm/` consume it.  10 of 10 flights, the counted table of §3
8. (done, then DROPPED BY THE OWNER) ego-planner, mission B.  Built, integrated, flown, dropped
   2026-10-15; the lessons are one section of `STATUS.md`
9. (done) **the ego-planner line removed from the tree** and the documentation merged back onto the
   pipeline that exists — 44 files deleted, `fsm/mission_fsm.py` 1759 -> 1014 lines,
   `src/tools/run-mission.sh` 416 -> 266
10. (done 2026-10-15) **hover at every corner, measured** — `RECT_HOLD_S=5`.  The producer advances ON
    ARRIVAL and the FSM accepts a leg mid-mission (§4.1); `logs/missionA-RECT7-hold5` gives **4 of 4 corners
    hovered >= 5.0 s** from `src/tools/dwell-audit.py` (c1 18.15 / c2 7.67 / c3 7.19 / c4 7.58 s), 4 of 4 again
    airborne-only, **5 of 5 B2 legs** and C2 0.562 m with 0 of 56696 samples inside the 0.20 m footprint.
    The four sorties that reported "1 of 4" are 0 of 4 under the airborne measure — that "1" was the
    aircraft parked on the ground at c1 after landing.  One sortie is not a rate: `STATUS.md` §"Next"
    item 1 asks for the batch

## 6. Rules (carried over from the rounds that produced the numbers)

1. Every number carries its frame, origin and window; angles are unwrapped before differencing.
2. No compensating patches: fix the physical cause, and prove it with a controlled A/B.
3. A parameter change must be verified to have reached the consumer.
4. Batch runs quantify rates; one flight is never a conclusion.
5. Every test must also be run against a deliberately wrong configuration and must fail.
6. All WSL work goes through script files (inline `bash -c` gets mangled by the Windows shell).
7. Pin and print revisions: ArduPilot tag, LIO commit, liblio.so md5, world seed.
8. **Measurements are taken from the recording, never from the producer's own log.**  MEASURED: the
   corner-hold producer logged three 5 s holds while the aircraft spent them somewhere else entirely, which
   is why `src/tools/dwell-audit.py` exists and why it reads `tap.csv`.

## 7. Documentation budget (a hard rule from the user, 2026-10-13)

**At most 5 markdown documents in `docs/`, ever.**  Currently 5:

1. `PLAN.md` — this file: goal, decisions, acceptance, order, rules
2. `ARCHITECTURE.md` — components, dataflow, topics, TF tree, transport, safety gates
3. `FRAMES.md` — coordinate conventions, the wire facts, where conversions live, the test matrix
4. `STATUS.md` — current state, evidence, next steps
5. `PLANNERS.md` — the `/plan/cmd` planner contract, who owns which safety duty, and how to swap a planner
   (added 2026-10-16 with the planner-swap work; the registry in `tools/docs-budget.py` carries it)

THE BUDGET IS NOW FULL.  A new subject does **not** get a new file: it goes into whichever of the five owns
it, and if a sixth file is ever created, another must be merged away in the same change.  This is
executable, and it is run before every commit:

```
cd /mnt/d/distill_self/lio_avoid && python3 src/tools/docs-budget.py
```

Figures, CSVs and the metrics dumps under `docs/` are evidence, not documents, and are not counted.

## 8. Open questions

1. **The corner hold, measured** (§5 item 10): **demonstrated** — 4 of 4 corners >= 5.0 s on
   `-RECT7-hold5`, on the truth tap, and 5 of 5 B2 legs.  Open: it is **1 sortie**; the repeat (3-5 flights,
   rate reported) is `STATUS.md` §"Next" item 1.
2. **B2's remaining gap**: 39 of 50 legs within 50 mm in the frozen batch; the failures are the turning
   legs, and the measured cause is a lightly damped lateral response of the position loop at 0.25 m/s, not
   the corner carry-over (rejected by experiment, `STATUS.md` row 32).  The corner dwell improved it on the
   one flight that has it: 5 of 5 legs, worst 29.1 mm.
3. Should the simulator still carry a compass (never an EKF source) so heading can be cross-checked?
4. The old `gazebo_apm` tree holds ~40 historical documents (the evidence chain for rounds 1-7).  It is a
   frozen archive rather than a living document set: fold it into the budget as well, or leave it frozen?
5. **DECIDED** — spawn heading North (#14), tap frame SITL-native NED/FRD (#13), altitude source (a) then
   (b) (#15), the 0.20 m contract (17), the freeze of the mission-A set (#16), and the corner hold's
   convention on the trajectory interface (§4.1, 2026-10-15).
