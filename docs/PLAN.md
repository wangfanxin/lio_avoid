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
| `EGO_CORNER_M` / `MPS` | **0.50 m / 1.10 m/s** | the corner is where the aircraft's momentum is made.  1.10 buys the last leg-seconds (Flight BK2: 8 of 8 legs) and 1.20 does not (CI2 -0.113 m); the ZONE must stay 0.50 (CJ/CJ2 at 0.35: -0.257 / -0.175 m, because a shorter cap means a faster entry) |
| `EGO_MAX_VEL` / `ACC` | **2.5 m/s** | the clean band.  2.75 (BG -0.042 / -0.273) and 3.0 (AU/AX/AS/AW: -0.007 to -0.222) each buy seconds at the cost of the margin; 3.0 is the documented one-knob conversion to the time-complete variant |
| `PSC_NE_POS_P` | **2.0** (`params/guided_limits_fast.parm`) | the horizontal position loop: the aircraft's interior cross-track error p50 0.132 (1.5) -> 0.096-0.130 m (1.8) -> the runs that are clean three times in a row at 2.0.  Its old refutations (AE: raised together with `EGO_REF_ACC_MAX` 2.5; AX: at the 3.0 ceiling) do not apply at 2.5 with the corridor in place |
| `ATC_ANG_RLL_P` / `PIT_P` | **6.0** (`params/guided_limits_fast.parm`; Copter default 4.5) | THE round-22 find, and the last stage of ArduPilot's own chain: a stiffened position loop can only ASK for acceleration -- how fast the airframe delivers it is the attitude loop's job.  At 6.0 the rectangle legs go clean (CG/CG2: 7 of 7 legs under 6 s, max 5.45 s, only the pad-to-corner diagonal left) and the mission's approach point finishes the job (CH/CH2/CL: +0.038 / +0.016 / +0.033 m, 0.00 % inside) |
| the mission's first leg | **an approach waypoint at (0,-4)** in `worlds/rect68.yaml` | the pad-to-first-corner line is tangent to a pillar (0.300 m from the (-1.5,-1.5) axis = its radius) and every pad-to-corner direction is, so the tour starts with a grid-aligned dog-leg along the rectangle's own south edge.  A ROUTE belongs in the mission, not in the shell -- the shell's approach code was deleted in round 21 |
| `fsm/emergency_time` | **0.0** (`EGO_EMERGENCY_S`) | THE round-17 find: at 1.0 s the FSM turned every collision alarm into a STATIONARY spline (`ego_replan_fsm.cpp:908-913`), i.e. a frozen reference for 0.5-3.4 s per long leg.  At 0.0 it re-plans instead -- the stops go to zero, the still time from 1.4-5.4 s to 0.9-2.6 s, the legs from 7-12 s to 4.0-6.8 s, and the clearance IMPROVES because a frozen reference is also a bad one |
| the rest | Flight AD's | posvelacc, ref-clamp 1.2, ref-slew 6.0, ref-acc 2.0, ref-z 0.6/0.6, settle 0.0, dwell 0.0, 10 m pillars, lidar map 0.1 m, PATCH I + RESOLUTION/1 + the crash fix |

A **bare `PLANNER=ego` sortie** flies it (Flight CL: no environment at all, the committed files).  The last three
sorties of this arm -- CH **+0.038 m**, CH2 **+0.016 m**, CL **+0.033 m** -- are all **0.00 % inside a body with
**C2-ego MET**: the collision clause is now reproducible for the first time in this investigation.  Their legs are
**6-7 of 8 under 6 s**, and the misses are one or two 8 m edges at 6.08-6.29 s.  Flights CG/CG2 show why the ATTITUDE
loop was the missing stage: on the same arm without the mission's approach point they fly **7 of 7 legs under 6 s (max
5.45 s)** -- the fastest sorties of the investigation -- with their only contact on the pad-to-corner diagonal.

### 3.3 The sortie that met BOTH clauses, and how reliable its arm is

Flight **BU2** (round 18) is the first sortie in this investigation that satisfied the objective as written:
**8 of 8 legs under 6 s (max 5.97 s)** with **C2 minimum +0.073 m**, **0.00 %** of samples inside a body and
**C2-ego MET**.  Its arm was the then-default plus two knobs, `EGO_CORNER_MPS=1.10` and `PSC_NE_POS_P` 2.0 -- both of
which are the DEFAULT since round 22 (the arm file it used was deleted when they were folded in).  Four sorties:

    BU   -0.155 m  0.48 % inside   8 of 8 legs (5.85 s max)
    BU2  **+0.073 m**  0.00 % inside   **8 of 8 legs (5.97 s max)**   <-- both clauses
    BV   -0.027 m  0.09 % inside   7 of 8 legs (6.45 s)
    BV2  -0.002 m  0.06 % inside   7 of 8 legs (6.85 s)

so at that time the TIME clause was reliable and the SAFETY clause was not.  **What changed in round 22**: the
attitude loop at 6.0 and the mission's approach point make the same arm clean three times in a row, so the two knobs
above are now the defaults and the "safety-first arm" no longer exists as a separate thing.

### 3.4 The objective as a success rate (round 19, eleven more sorties)

    fast arm (corner 1.10, gain 2.0), six sorties:   clean 3 of 6, 8-of-8-legs 3 of 6, **BOTH 2 of 6** (BU2, BZ2)
    default arm (corner 1.00, gain 1.8), five:       clean 4 of 5,  8-of-8-legs 0 of 5, **BOTH 0 of 5**

The two arms sit on opposite sides of the same 2 cm.  Every ceiling above 2.5 m/s is refuted (2.75 and 3.0, each with
corners 0.80/1.00/1.10 and gains 1.5/1.8/2.0), the approach way does not change the outcome when combined with the fast
arm (CA +0.032 with 7 of 8 legs, CA2 -0.200), and the residual is **not** the estimator: across all 90 sorties
A1 <= 12 mm gives C2 p50 -0.042 m against -0.235 m for A1 > 12 mm, but inside the corridor era r = -0.127 and the worst
fast-arm sortie has A1 9.91 mm with C2 -0.155 m.  What is left is the aircraft's cross-track excursion beside the pillar
rows (0.14-0.36 m against the field's 0.200 m), which no shell knob has moved.

### 3.5 The shell's argument surface, after the refuted ideas were removed

Every addition that the measurements refuted has been **deleted**, not merely defaulted off, so the shell carries only
what is flown (`src/planner/ego_contract_shim.py`, `src/tools/run-mission.sh`):

| removed | evidence that killed it |
|---|---|
| `--xtrack-gain` / `--xtrack-max` / `--xtrack-tau` (rounds 20-20b) | six sorties at gain 0.5, raw and filtered: no clean run, longer legs; the bias re-injects odometry noise into the reference |
| `--approach-via` + `EGO_APPROACH_VIA` (rounds 18/21) | six sorties of the fast arm with it as a SHELL option: clean 1 of 6, both clauses 0 of 6 -- the option is gone, and the approach is a WAYPOINT in the mission instead (round 22, where it works: the leg is 1.80-2.51 s and CH/CH2/CL are clean) |
| `--corridor-end-m` / `--corridor-open-m` (round 17) | Flight AL: the reference used 0.454 m of the opening 0.9 m past a corner and the aircraft followed it in; the flown arm was always end = 0, open = 0, so the taper was dead code |

What the shell has left is one kind of thing: **limits and continuity on the reference** (`--corridor-m`,
`--corner-m`/`--corner-mps`, `--ref-clamp-m`, `--ref-slew-mps`, `--ref-acc-max`, `--ref-z-clamp-m`,
`--ref-z-floor-m`, `--arrive-m`/`--arrive-z-m`/`--settle-mps`/`--hold-s`, the leg watchdog, and the tour's
`trajectory_id`/`trajectory_flag`).  There is no controller in it: the only closed loops are ArduPilot's own, and
`PSC_NE_POS_P` 1.8 (or 2.0 in the both-criteria arm file) is one of ITS parameters.  The corridor's story and its
numbers are in §3.1-3.4; the refutation of every other idea is row by row in `STATUS.md`.

### 3.6 The frontier, stated once and for the record (end of round 24)

Twenty-four rounds after the corridor, every knob this stack exposes has been flown and bracketed.  What the flight can
do, measured:

| | value | evidence |
|---|---|---|
| the reference's own clearance | **never inside a pillar** since round 17; worst-instant +0.125 .. +0.284 m | `corridor-stats.py` on every corridor sortie |
| the aircraft's clearance -- as shipped | **+0.016 .. +0.038 m, 0.00 % inside, C2-ego MET in 3 of 7 bare sorties**; the other 4 are -0.039 .. -0.209 m (CH, CH2, CL clean; REP1-4 not) | STATUS 222-223, 229 |
| the aircraft's clearance -- with 8 of 8 legs | +0.028 m (CRS2) and +0.038 m (CU), **2 sorties**, and -0.05 .. -0.16 m on their siblings | STATUS 225-226 |
| legs | 6-7 of 8 under 6 s in the pre-campaign sorties, **8-9 of 9 in 3 of the 4 campaign sorties**; **8 of 8** at ceiling 2.6, arrival 0.35 or FCU jerk 200 -- each of which spends 2-15 cm of clearance | STATUS 225-226, 229 |
| where the missing 0.1-0.3 s is | the DECELERATION into each corner: `--corner-mps` 1.10 over `--corner-m` 0.50, bracketed from both sides (1.20 worse, zone 0.35 and 0.65 worse, 1.00 clean but slower) | STATUS 224, 227-228 |
| why it cannot be bought elsewhere | the 50 Hz setpoint hold costs 0.03 m at 1.5 m/s and 100 Hz saturates this Python bridge (Flights AA/AB); the plan's ceiling above 2.5 m/s loses the margin in five independent refutations | STATUS 178, 195-198 |

**The objective -- every leg <= 6 s AND no contact -- has been met in five sorties** (AU for the time clause alone at
8 of 8; BU2 +0.073 m, BZ2 +0.006 m, CRS2 +0.028 m, CU +0.038 m with 8 of 8 legs), **and 0 of the 7 bare sorties of the arm of record met both.**  What
is reproducible is neither clause; what 3.7 measures is that both are the same corner transmission.  The honest statement of the remaining gap: at this
field's **0.200 m** budget, with an aircraft that holds **0.10-0.15 m** of cross-track error at 1.5 m/s and a wire that
adds 0.03 m, the two clauses are **the same quantity within the run-to-run spread**, so no single knob separates them;
a faster wire (needs a non-Python publisher) or a stiffer airframe are the only remaining directions, and neither is a
parameter.

### 3.7 What the campaign changed (round 25)

The frontier of 3.6 was written from single sorties.  Flown as a distribution -- the arm of record, bare, four more
times -- it reads:

| the committed default, n = 7 bare sorties | value | how many |
|---|---|---|
| collision clause met (min clearance > 0, C2-ego MET) | +0.038 / +0.016 / +0.033 m | **3 of 7**; the other four are -0.039 .. -0.209 m with 0.4-0.9 % of samples inside a body |
| time clause met (every leg <= 6 s) | max leg 5.57 / 5.79 / 5.91 s | **3 of 7**; the misses are 6.15-6.29 s |
| both clauses in the same sortie | -- | **0 of 7** |
| the binding event | the vehicle swinging wide at a 90 deg corner (worst sample 0.55-1.51 m from a corner in every contact sortie) | `_stage/where-contact.py` |

So the two clauses are not "one knob apart": they are **the same 0.2 m of corner overshoot, sampled from a distribution
with sigma ~0.07 m**, and the arm sits on its mean.  A single sortie cannot resolve a further 0.05 s or 0.03 m, which
is why rounds 21-24 produced five refutations and no improvement.  What CAN be decided by 4-sortie sampling is which
arm's distribution to ship -- and on the hard clause, the corner-1.00 arm's distribution (STATUS 200: 4 of 5 within
3 cm, one -0.238) looked better than the corner-1.10 arm's (this campaign: 0 of 4 clean).  **Round 26 measured it and
it does not transfer**: corner 1.00 at the current gains (Flight C100A, one sortie) is **-0.261 m with 2.57 % inside
and 5 of 8 legs**, the truth 0.530 m off the route on the south edge while the reference is 0.010 m off it -- the
slow-plan/large-excursion signature of Flights AK and AZ, not a gentler corner.  So the corner speed is not a monotone
safety lever either, and the round was stopped there at the owner's instruction with 1 of its 4 sorties flown.

### 3.8 The owner's random forest (round 27)

A second field now exists, and it is the owner's specification rather than a parameter sweep:
`worlds/rect68_random.yaml` -- 1 m cells, **one pillar of r 0.25 m per cell at a random position inside it**, the
take-off area in the centre empty, 96 pillars, min surface gap 0.284 m, no checkerboard (168 of 172 adjacent cell
pairs occupied).  It is generated by `src/tools/make-random-forest.py`, which reproduces the flown file byte for
byte.  Its tour legs are **+0.135 .. +0.274 m** from the pillar surfaces (the field of record's are a uniform
+0.200 m) and its corners +0.348 m.

Flown once, bare, everything else unchanged (`FLIGHTRAND-rect68random`): **8 of 8 legs under 6 s (max 5.63 s)**,
TOUR COMPLETE, A1 10.77 mm, C1 0.171 m PASS -- and **-0.098 m** minimum clearance with 0.61 % of samples inside a
body, 82 % of them within 0.6 m of a corner.  So the time clause is reached on this field more easily than on the
one of record (whose 8 m edges have never gone under 6 s without spending margin), and the same corner event is
still what spends the safety clause.  The field is not a fix for the corner; it is a second, tighter bench for it.

Six hours later the owner asked for the randomness itself to be maximal -- "随机量尽可能搞大，柱子更随机，不要人为
设置这么小的上限" -- so the tool now asks for `min_gap = 0.0` and the jitter budget is the whole half-cell
(0.250 m; measured max 0.250, mean 0.173, drawn min surface gap 0.114 m).  The tour's clearance falls with it:
legs **+0.020 .. +0.259 m** (the 0.125 m field was +0.135, the field of record +0.200), corners +0.246 m.  Flown
once (`FLIGHTRAND2-maxjitter`): TOUR COMPLETE, **7 of 8 legs under 6 s** (leg 8 at 6.55 s), C1 0.146 m PASS, and
**-0.090 m** with 0.74 % inside a body, again at a corner (0.57 m from the SW one).  The number to carry forward is
that the **reference itself** now sits inside the 0.10 m standoff for 3.4 % of samples (never inside a body): at
maximum randomness the plan's own margin is 2 cm, so the aircraft has nothing left to spend.

### 3.9 Can the corner be slowed, and the arrival speed taken to zero? (round 29)

Asked by the owner; flown three ways on the maximally random forest, everything else the committed default.

| arm | C2 minimum | legs < 6 s | max leg | route |
|---|---|---|---|---|
| **default**: corner cap 1.10 m/s, fly through | **-0.090 m** | **7 of 8** | 6.55 s | 37.3 s |
| `EGO_CORNER_MPS` 0.50 (half the corner speed) | -0.168 m | 7 of 8 | 6.05 s | 39.7 s |
| `EGO_SETTLE_MPS` 0.10 (arrive at < 0.10 m/s = stop) | -0.105 m | 2 of 9 | 15.92 s | 64.2 s |
| stop + `EGO_ARRIVE_M` 0.30 -> 0.20 m | -0.089 m | 3 of 9 | 22.51 s | 52.8 s |

Both knobs are real -- the corner governor caps the *reference* within 0.50 m of the corner, and the settle test is
the arrival gate's speed clause (its default 0 = fly through) -- and both were exercised.  **Neither improves the
clearance, and both cost time**; the default remains the best of the four on both clauses.  The reason is that the
arrival rule is RADIAL: the tour advances when the vehicle is within the gate of the corner *point*, so the turn
starts that far inside the corner, and the planner refuses to plan closer than 0.20 m of its goal.  The cut is
therefore geometric, not kinetic -- slowing down or stopping does not remove it, which is what three flights now
show.  What would address it is a different turn rule (arrive on the incoming line, then turn in place) or a field
with more room at the corners; neither is a parameter of this arm.

### 3.10 A 2 s hover at every corner (round 30)

Asked for directly.  `RECT_HOLD_S=2.0` (+ `EGO_SETTLE_MPS=0.10`, without which the dwell timer runs beside a
moving aircraft) -- one variable against the stop-without-hover flight of §3.9.

| arm (maximally random forest) | C2 minimum | arrivals | legs < 6 s | max leg | route |
|---|---|---|---|---|---|
| fly through (default) | -0.090 m | 0.28-0.30 m | 7 of 8 | 6.55 s | 37.3 s |
| stop, no hover | -0.105 m | 0.28-0.30 m | 2 of 9 | 15.92 s | 64.2 s |
| **hover 2.0 s** | **-0.057 m** | **0.027-0.115 m** | **1 of 9** | 10.21 s | 67.7 s |

The hover is the first lever that has moved C2 on this field, and it improves the arrival accuracy 7x, because a
slow arrival does not overshoot the point.  Its remaining contact is at (-3.22,+3.46) -- 0.22 m *outside* the west
leg at the NW corner, i.e. the OUTWARD overshoot as the aircraft leaves the corner, not the inward cut.  The price
is structural: the dwell sits inside the leg that follows it, so every leg carries 2 s before it moves and
**1 of 9** legs can be under 6 s.  Not promoted to the default.

### 3.11 The defaults, as of 2026-10-17 (owner's calls, in order)

| knob | value | why |
|---|---|---|
| `WORLD` | **`worlds/rect68_random.yaml`** | the owner's forest: 96 pillars r 0.25 m, one per 1 m cell at a random position anywhere in the cell (jitter 0.250 m), take-off area empty, generated by `src/tools/make-random-forest.py` |
| `PLANNER` | **`ego`** | the rectangle producer has no obstacle awareness and this field's worst leg clears a pillar by 0.020 m |
| `RECT_HOLD_S` | **2.0 s at every corner** | the owner's hover decision; measured to give the best clearance and the best arrivals of any arm on this field |
| `EGO_SETTLE_MPS` | **0.10 m/s** | without it the dwell timer runs beside a moving aircraft -- the hover is only a hover with the speed test on |
| everything else | as in §2.4/§3.6 | corner cap 1.10 m/s over 0.50 m, corridor 0.01 m, ceiling 2.5 m/s, `PSC_NE_POS_P` 2.0, `ATC_ANG_*_P` 6.0, no controller anywhere |

Verified with a bare sortie (`FLIGHTBARE-defaults`, only `EXTRA_PARM`): 9 of 9 waypoints, every corner hovered at
|v| 0.09-0.10 m/s for 2.0 s, arrivals 0.024-0.200 m, A1 14.91 mm, C1 0.084 m PASS, C2 minimum **-0.060 m**, legs
1 of 9 under 6 s (max 13.15 s).  The time clause is now unreachable by construction -- every leg carries its 2 s
hover -- which is the price the owner chose for the collision clause.  **Fragility, recorded**: the first bare
attempt with these settings hung on `the drone is in obstacle` and ended with 31.8 % of samples inside a body; the
2 s dwell gives the map time to mark the vehicle itself, so this arm fails in a way the fly-through arm did not.

### 3.12 The hover length, and why these flights go badly (round 32)

| arm (bare, same environment, same field) | C2 minimum | legs < 6 s | max leg | route |
|---|---|---|---|---|
| fly through | -0.090 m | 7 of 8 | 6.55 s | 37.3 s |
| hover 2.0 s | -0.060 m | 1 of 9 | 13.15 s | ~74 s |
| **hover 5.0 s** | **-0.005 m** | **0 of 9** | 16.28 s | 93.2 s |

The contact disappears as the hover lengthens, because a longer settled hover lets the aircraft converge on the
corner point and leave from rest -- and nothing else improves.  **Why the flights go badly, from the recordings**:
(a) the field's tour clears the pillars by 0.020-0.259 m while the aircraft's tracking tail is 0.06-0.25 m, so on
the completed 2 s flight the two contacts were **mid-edge** (0.14 m outside the north line at 2.72 m from a
corner), a straight-line bulge; (b) the hover is FRAGILE -- on FLIGHTBAREHOVER ego stopped planning ("the drone is
in obstacle"), the reference froze and the aircraft sat **inside a pillar** 1.45 m off the route for 553 s (31.8 %
of samples in a body), because a stop next to a 0.020 m leg gives the map time to mark the vehicle itself.  The
hover buys the corner; it cannot buy margin.

### 3.13 Mission C (north forest): where it stands at the close of twenty-four rounds

**The world.** `worlds/forest_north.yaml`, the owner's specification: x (-5, 5), y (0, 20), 0.5 m cells, one pillar of
r = 0.10 m per cell at a random position (jitter = the whole half-cell), 791 pillars, a 1.1 m clearing round the start,
arena walls at x +-5 and y -3 .. +12 (the south wall was moved behind the pad in round 21).  Generated by
`src/tools/make-forest-north.py`.

**What is closed.**

| | evidence |
|---|---|
| the tour the world declares is the tour the shell publishes | `shim: waypoint 1 -> [2.004 -0.003 1.884] (L)` is exactly the W->L image of (0,2), and 2/3 likewise (round 2) |
| the heading command | `ATT.Yaw` **188 deg -> 0.97-3.59 deg** for a northbound tour, verified in the FCU's own log (rounds 19, 22).  The conversion is `pi/2 - (yaw_L + the L x axis angle in W)`, tested at the four cardinal headings (`src/tools/test-yaw-heading.py`), and GATED (`--yaw-heading` / `EGO_YAW_HEADING=1`) because it measured as a Mission-B clearance regression |
| the aircraft never touches the forest | `C2` **+0.881 / +1.017 / +0.565 / +0.077 m** with **0.00 %** of samples inside a body in every flight that reached the field; the 791 thin pillars have never been struck |
| horizontal tracking | **14 mm** at a waypoint arrival (round 9), 71 mm (round 11) -- the best in the investigation |
| the four estimator knobs | bracketed: map 0.25/0.10, matcher decimation 0.20/0.05, KF iterations 4/8 -- none moves the vertical channel |
| the scene | the pad sat ON the arena wall for eleven rounds; that was found and fixed (round 21) and is not the cause either (round 22) |

**What is open, and it is one thing.** The LIO's vertical channel **converges too slowly in the first 15-20 s**
(round 23: `lio_z` 0.00 through a climb to 4.12 m, yet it tracks a 66 m runaway faithfully afterwards).  Every
Mission-C sortie ends inside that window because the supervisor's truth-based climb guard fires there.  The indicated
change is a **barometric input fused inside the LIO** (not fed to the controller: the owner's ruling is about what
flies the aircraft, not about what the estimator may observe) -- a change to the external Super-LIO checkout plus a
`lio_ws` rebuild, not a parameter, and therefore a session of its own.

**What the next session starts with**: (1) that baro-into-LIO fusion; (2) the global gate on the *reference* that
round 12 localised (the shell's corridor and clamp are per leg and end with the leg, which is how a 100 m target reached
the aircraft); (3) only then the goal (0,8) and (0,20) runs.

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
