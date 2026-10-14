# ARCHITECTURE — boxes, links, topics, frames, interfaces, safety

One of the project's four living documents (budget in `PLAN.md` §6).  Everything here is a contract:
rates, frames and interfaces come from measurements or from the firmware source, and each conversion has
a test named in `FRAMES.md` §5.

**No Gazebo anywhere.**  The lidar is computed by our own analytic ray caster, the IMU is generated from
the simulator's true state, and RViz is used for display and for editing the world.  Where the old
Gazebo-based pipeline had `/mid360/*` (Gazebo Transport), `gz_livox_bridge`, `/sensors/*`,
`/uav/*` and orphan parameter files, this pipeline has exactly one interface per function and no
orphans -- a parameter file that nothing loads is a bug (verifiable by grep).

---

## 1. The boxes

```
┌══════════════════════════════════════════════════════════════════════════════════════════┐
║ BOX 1   SITL                                                                             ║
║         ArduPilot Copter-4.7.1, built-in quad dynamics, EKF3, GUIDED controller          ║
║         owns: the true state, the FCU's local NED frame, the flight modes                ║
║   ┌──────────────┐    ┌───────────────┐    ┌──────────────────────────────────────────┐ ║
║   │ quad model   │───►│ built-in IMU  │───►│ EKF3   (EK3_SRC1_* = 6, LIO only)        │ ║
║   │ (its physics)│    │ GPS/baro/comp │    │ GUIDED (position/velocity setpoints)     │ ║
║   └──────┬───────┘    └───────────────┘    └────────────────┬─────────────────────────┘ ║
║          │ true state                                       │                             ║
║          └──────────────► TRUTH TAP (sim-only, env-gated) ──┼──► UDP 1200 Hz             ║
╚═════════════════════════════════════════════════════════════╪═════════════════════════════╝
        ▲ MAVLink 5760/udp:  setpoints 84 @20-50 Hz, commands 76        │ truth: p/v NED,
        │                    telemetry 31/32 @20-50 Hz, 0/253 @1-2 Hz   │ q FRD, gyro, accel
        │                                                               ▼
┌═══════╪═════════════════════════════════════════════════════════════════════════════════════┐
║ BOX 2 │  sim/           the world, the sensor, the clock, the world editor                 ║
║   ┌───┴──────────┐   ┌──────────────────┐   ┌─────────────────┐   ┌────────────────────┐  ║
║   │ truth_rx     │──►│ lidar_imu        │   │ clock           │   │ world editor       │  ║
║   │ (UDP -> ROS) │   │ raycast + noise  │   │ /clock 100 Hz   │   │ RViz clicks ->     │  ║
║   │ 1200 Hz      │   │ IMU model        │   │ /sim/rtf 1 Hz   │   │ worlds/*.yaml      │  ║
║   └──────────────┘   └────────┬─────────┘   └─────────────────┘   └─────────▲──────────┘  ║
║   $WORLD (ONE file) ──────────┘  (geometry, mount 45 deg, mission; the SENSOR gets --world too, so
║   generation and judgement cannot disagree -- see 4.0 rule 1)                  │           ║
╚════════════════════════════════╪══════════════════════════════════════════════╪═══════════╝
     /livox/lidar 10 Hz (S)      │      /livox/imu 200 Hz (S)                   │ /clicked_point (W)
     /sim/truth/odom 100 Hz (W)  │                                              │
                ┌────────────────┴────────────────┐                             │
                ▼                                 ▼                             │
┌═══════════════════════════════════════════════════════════════════════════════╪═══════════┐
║ BOX 3   lio/          one LIO process (Super-LIO or FAST-LIO) + adaptation    │           ║
║   in : /livox/lidar 10 Hz (S), /livox/imu 200 Hz (S)                          │           ║
║   out: /lio/robo/odom 200 Hz (L, body pose)   /lio/odom 10 Hz (L, WITH velocity)│         ║
║        /lio/imu/odom 200 Hz (L, IMU state + twist)                             │           ║
║        /lio/cloud_world 10 Hz (L) = THE VOXEL MAP (0.25 m), not a raw scan     │           ║
║        /lio/path (L)                                                          │           ║
╚═══════════════╪═══════════════════════════════════════════════════════════════╪═══════════╝
                │ /odom 200 Hz (L)          /cloud_registered 10 Hz (L)         │
                ├──────────────────────────────┬───────────────────────────────┘
                │                              ▼
                │            ┌═════════════════════════════════════════════════════════════┐
║ BOX 4   planner/  = ego-planner (planner_ws, patches 0002..0008) + ego_adapter.py        ║
║   in : /lio/robo/odom 200 Hz (L)    <- state + map centre                                ║
║        /cloud_registered 10 Hz (L)  <- RELAYED by tools/scan_relay.py from /livox/lidar  ║
║              + /lio/robo/odom: RAW registered points, publisher QoS RELIABLE             ║
║        $OUT/stations.yaml (L)       <- fsm/waypoint* written ONCE at launch by           ║
║              tools/mission-params.py: the stations are EGO's, advanced on the VEHICLE    ║
║        /goal_pose (W) -> L          <- legacy goal path; DEAD under fsm/flight_type=2    ║
║   out: planning/bspline (L) -> the adapter re-projects it onto the vehicle, gates the    ║
║              arena and the 0.20 m footprint, WITHHOLDS an expired spline WITH A REASON   ║
║        /planner/traj (nav_msgs/Path, L)  +  wp_index (Int32, change + 1 Hz keep-alive)   ║
                │            ╚═══════════════════════════════════╪═════════════════════════╝
                │                                             │ /planner/traj (L) + wp_index
                ▼                                             ▼
┌══════════════════════════════════════════════════════════════════════════════════════════┐
║ BOX 5   fsm/  +  bridge/        scheduling, frames, arrival, FCU communication            ║
║   in : local_pose (msg 31/32 -> F, ENU labels, z up) @20-50 Hz      [bridge]              ║
║        state (HEARTBEAT/STATUSTEXT or /mavros/state) @1-2 Hz        [bridge]              ║
║        /odom 200 Hz (L)            <- for logging and the sanity gate                     ║
║        /planner/traj 4 Hz (L)      <- run ON THE SPLINE'S OWN CLOCK (per-point stamps)    ║
║        /mission/stop (Int32)       <- WHERE THE MISSION IS: the observer sequencer        ║
║                                       mirrors ego's wp_index here (-1 = tour over)        ║
║        /lio/status 1 Hz            <- gate: rate, jump, degeneracy                        ║
║        worlds/*.yaml mission (W)   <- the mission it schedules                            ║
║   out: setpoints (setpoint 84) @20-50 Hz   [bridge: normalised F -> NED on the wire]      ║
║        external nav (msg 102/331) @20-100 Hz  [bridge: LIO -> FCU]                        ║
║        /fsm/phase (String) on change + 1 Hz                                               ║
║        tf: world_w -> odom  (static, from Calib, measured once per flight)                ║
║   owns: the world-contract hold (the straight segment must clear 0.20 m), the truth-fed   ║
║         geofence, and a BOUNDED abort that NAMES the mechanism instead of going silent    ║
╚═══════════════════════════════════════════════════════════════════════════════════════════╝

┌───────────────────────────────┐        ┌──────────────────────────────────────────────────┐
│ BOX 6   RViz                  │        │ BOX 7   tools/                                   │
│  shows: /sim/world/markers(W) │        │  recorder: every stream, both stamps -> logs/     │
│         /cloud_registered (L) │        │  metrics: A1-A4 / B1-B4, counts of diverged       │
│         /lio/map, /tf, /odom  │        │  replay: the same pipeline from a recording       │
│         /planner/vis/*, /fsm/phase      │  docs-budget: the <=5 rule, executable           │
│  edits: /clicked_point (W)    │        └──────────────────────────────────────────────────┘
│  goal : /goal_pose (W)        │
└───────────────────────────────┘
```

Frame shorthand: **W** world ENU (the file, origin = pad) · **L** LIO world (z up, yaw arbitrary, origin
= LIO init) · **F** FCU local (NED on the wire) · **B** body FLU · **S** MID-360 housing.
L and F are the same frame up to axis labels; W differs from L by a known constant rotation + offset.

## 2. Where the lidar and the IMU come from

**They are generated by us.**  With no Gazebo there is no sensor plugin, so `sim/` synthesises both from
two inputs: the **true vehicle state** (SITL's built-in quad model, read through the truth tap) and the
**world geometry** (`worlds/pillars.yaml`).

```
BOX 1 SITL: built-in quad model
        true state at each model step: p, v, q (body->NED), body rates w, specific force a
                 │
                 │ truth tap (sim-only, env-gated, ~30 lines in libraries/SITL/)
                 │ UDP 1200 Hz (the model rate), ASCII, frame declared in the header: NED / FRD
                 ▼
BOX 2 sim/truth_rx ──► the true pose at the instant a scan starts (interpolated)
                 │
      ┌──────────┴────────────────────────────────────────────────┐
      ▼                                                           ▼
 LIDAR  (sim/lidar_imu)                                     IMU  (sim/lidar_imu)
   rays = the real MID-360 pattern, 20 000 rays per 100 ms     gyro  = w_true + bias + noise
          (scan_mode/mid360.csv, non-repetitive, 200 kHz)      accel = a_true + bias + noise
   hit  = closed-form intersection with cylinders /                (a_true is the specific force,
          floor / ceiling / walls                                  so gravity is already in it)
   noise = range sigma 0.02 m, blind 0.10 m, max 40 m          rate  = 200 Hz
   rate  = 10 Hz, per-point time offsets kept                  frame = S (the housing)
   frame = S, mount applied to the ray directions
      │                                                           │
      └──────────► /livox/lidar (PointCloud2)   /livox/imu (Imu) ◄──────────┘
                    both frame S, both into BOX 3 (the LIO)
```

Parameters live in `worlds/pillars.yaml`: `sensor:` (pattern file, mount, scan rate, points per scan,
range limits, noise, blind zone) and `imu:` (rate, noise sigmas, biases, `gravity_norm`).

**What we deliberately do NOT do**: read the FCU's own IMU over MAVLink (`RAW_IMU` / `HIGHRES_IMU`) and
hand it to the LIO.  That is the flight controller's sensor, at a lower rate, with SITL's own noise -- it
is not the MID-360's IMU, and it is not the sensor the real drone's LIO would see.

**A real difference from the old pipeline, recorded so it is a choice and not a surprise.**  In the old
Gazebo chain the FCU and the LIO consumed the *same* IMU: the Gazebo model's sensor fed both the
ArduPilot plugin and the LIVOX bridge.  Here there are two: SITL's built-in model generates the FCU's IMU
internally, and `sim/` generates the LIO's.  They derive from the same true motion, so both estimators see
consistent dynamics, but the noise realisations differ.  If that ever matters -- it would appear as an
attitude mismatch between `XKF1` and the LIO -- the fix is to make our IMU the single sensor and feed it
to SITL as well, which needs the external-sensor path (SIM_JSON) instead of the built-in model.

**Before the tap exists** the chain can already run: `worlds/pillars.yaml` carries a `fake_truth` block,
so `sim/` can fly a scripted square and publish exactly the same topics.  That is how steps 4 and 5 of
`PLAN.md` get built and tested without touching SITL.

## 3. Per-box cards (what each box subscribes, publishes, owns)

| box | subscribes (rate, frame) | publishes (rate, frame) | owns |
|---|---|---|---|
| 1 SITL | MAVLink: setpoints 84, commands 76, external nav 102/331 | MAVLink: 31, 32, 0, 253; truth tap UDP **1200 Hz measured** (one line per model step) | the true state, the EKF, the modes, the FCU local frame |
| 2 `sim/` | truth UDP 1200 Hz; `/clicked_point` (W); `worlds/*.yaml` | `/livox/lidar` 10 Hz (S), `/livox/imu` 200 Hz (S), `/clock` 100 Hz, `/sim/rtf` 1 Hz, `/sim/truth/odom` 100 Hz (W), `/sim/world/markers` 1 Hz (W) | the world, the sensor models, the clock authority |
| 3 `lio/` | `/livox/lidar`, `/livox/imu` | `/lio/odom` 10 Hz, `/odom` 200 Hz, `/lio/cloud_world` (the VOXEL map) 10 Hz (all L), `/lio/map`, `/lio/status`, `/tf` | the LIO world frame L and its odometry.  `/cloud_registered` is NOT its output -- `tools/scan_relay.py` builds that from the raw scan |
| 4 `planner/` | `/lio/robo/odom`, `/cloud_registered` (relayed from the raw scan), `$OUT/stations.yaml` (`fsm/waypoint*`), `/goal_pose` (legacy, dead under `flight_type=2`) | `/planner/traj` (`nav_msgs/Path`, L), `wp_index` (`std_msgs/Int32`), `/planner/stalled` (`std_msgs/Bool`) | the trajectory **and, since patch 0008, the station sequence itself** -- ego advances on a test on the VEHICLE, so nothing outside can run ahead of it |
| 5 `fsm/`+`bridge/` | FCU pose/state, `/odom`, `/planner/traj`, `/mission/stop` (the observer sequencer's mirror of `wp_index`), `/lio/status`, the mission file | setpoints, external nav, `/fsm/phase` | scheduling, arrival, the gates (world-contract hold, geofence, bounded abort), every conversion |
| 6 RViz | everything visual | tool topics | nothing (display and world editing only) |
| 7 `tools/` | everything | logs, tables, figures | the evidence |

## 4.0 The chain AS BUILT — 2026-10-14 (read this before the diagram: that one is the design intent)

Every row is a measured number from a run directory, and the row says where.

| stage | what really happens | the number, and where it came from |
|---|---|---|
| truth | ArduPilot SITL (Copter-4.7.1 + `patches/0001` tap) steps at 1200 Hz and emits `TAP` lines on UDP | **1200.5 Hz**, `logs/s3-parked-*` |
| sensor | `sim/lidar_imu.py` renders `/livox/lidar` (20 000 points, 10 Hz, per-point times) and `/livox/imu` | **199.4-200.1 Hz IMU**, decimation derived from the rate that ARRIVES, `logs/missionA-imu200a` |
| LIO | clean upstream Super-LIO `f89f48d`: `/lio/robo/odom` (200 Hz, body pose, L), `/lio/odom` (10 Hz, WITH velocity), `/lio/imu/odom` (200 Hz), **`/lio/cloud_world` = the VOXEL map (0.25 m)**, `/lio/path` | 200.1 Hz delivered, `logs/missionA-stepB10-*` |
| FCU feed | the FSM re-samples the newest LIO pose in its own 50 Hz loop and streams ODOMETRY (331) in NED/FRD | the FCU's own log: **VISP/VISV at 48.2 Hz**, `tools/bin-extnav-rate.py` |
| EKF | `EK3_SRC1_POSXY/POSZ/VELXY/VELZ/YAW = 6` (external nav), no GPS, no compass; every measurement and process noise set to the MEASURED value in `params/ekf_trust_lio.parm`, with `FS_EKF_THRESH` raised in proportion (it is a RATIO) | `tools/bin-params.py` on the flight BIN |
| Mission A | `planner/rectangle_source.py` publishes the rectangle on `/planner/traj`; the FSM visits its **corners** (`--follow corner`, the default and what every table in `docs/metrics/` was measured with) | A1 4.65-4.90 mm, A4 18.6-21.1 mm, 10 of 10 flights, `docs/metrics/stepB-rectangle-20261013.txt` |
| Mission B | ego-planner (`ZJU-FAST-Lab/ego-planner-swarm`, `ros2_version`, `23a8d5a`, GPL-3.0, + `patches/0002..0008`) plans in L and publishes `planning/bspline`; **`planner/ego_adapter.py`** turns it into `/planner/traj` (Path, frame `lio_world`, >= 2 Hz, per-point stamps), RE-PROJECTS it to start at the vehicle, gates the arena and the 0.20 m footprint against the world file, and WITHHOLDS an expired or degenerate spline WITH A REASON | offline: 0.4538-0.4978 m clearance, 1 withheld of 68, goal round trip 0.000000 mm, `docs/metrics/ego-offline-*.txt`; the withhold, from AO2: ONE expired spline re-projected ~250 times as a 2-pose 0.006 m stub that the consumer refused every time, `docs/metrics/phaseB-ao2-arrival-is-the-vehicle-20261014.txt` |
| the planner's map | the planner declares `/cloud_registered`; **the LIO publishes `/lio/cloud_world`** -- so both the NAME and the QoS needed bridging: `tools/scan_relay.py` registers the RAW scan into L (`/livox/lidar` + `/lio/robo/odom`, no voxels) and publishes it **RELIABLE** | the planner's own log: `offering incompatible QoS. No messages will be sent to it`; the voxel map's own closest point to a pillar was **-0.244 m** (a voxel centre sits up to half a voxel inside), which is why raw points are relayed instead |
| Mission B flying | the FSM follows each plan ON THE PLAN'S OWN CLOCK (`--follow path` / station loop, no rebasing) and owns the world-contract hold and the bounded abort | AF1: **8 of 12 stops REACHED** and landed, but C2 min **-0.376 m** with **142220 of 305081 samples (46.62 %)** inside the 0.20 m footprint; AO2: **1 of 12 stops**, C2 min **0.259 m** with **0 of 84620** inside, A1 3.16 / A2 3.90 / A4 24.71 mm, C1 0.010 m, ABORT at t = 82.5 s WITH a reason |
| the station sequence (2026-10-15) | **ego owns it now**: `tools/mission-params.py` writes `fsm/flight_type=2` (PRESET_TARGET) + `fsm/waypoint_num` + `fsm/waypointN_{x,y,z}` + `fsm/waypoint_advance_m` (0.300 m) + `fsm/waypointN_dwell_s` + `fsm/waypoint_loop=false` + `fsm/waypoint_start_alt` (1.28 m) into `$OUT/stations.yaml`, which `tools/run-planner.sh` passes as a second `--params-file`; the stops are the mission's own (`tools/world-circle.py::commanded_points`), converted ONCE into L with the run's measured calibration.  The keys must be FLAT and quoted (`"fsm/waypoint_num": 12`): ROS 2 flattens a nested map to `fsm.waypoint_num`, which the C++ name `fsm/waypoint_num` never matches -- measured once, the file was on disk and ego armed with every key at its default (`sequence armed: -1 waypoints, advance 0.150 m`, `flight_type` still 1) and published nothing (`79b4bec`) | round trip W->L->W **0.000000 mm** and **0.1 mm** agreement with conversions logged IN FLIGHT by the independent implementation (stops 0/1/2 against AO1/AO2's GOAL-TRACE and adapter lines); `fsm/waypoint_advance_m` 0.300 m against AO2's measured **0.273 m** settling distance, because upstream's 0.15 m rule would never advance; **FLOWN** (AP1/AP3) -- `WP-INDEX: sequence armed: 12 waypoints, advance 0.300 m, loop 0` and `starting the tour at z=1.28 m (gate 1.28 m, 12 waypoints)` in ego's own log, with the aircraft settling **0.330 / 0.343 m** from stop 1, i.e. just past the 0.300 m tolerance |
| the sequencer, demoted to an observer (2026-10-15) | `planner/goal_sequencer.py --observe-wp-index /wp_index` publishes NO goals and no index of its own: it mirrors ego's `wp_index` onto `/mission/stop` (Int32, -1 = tour over) and still SCORES the stops from truth | AO1, driven from outside: the sequencer walked stops 2..11 on its own 45.2 s timeout while the vehicle hovered at stop 1 -- **1 of 12** stops reached.  **FLOWN as an observer** (AP1/AP3): `goal published 0 time(s) for it`, and it still scored `stop 1/12 REACHED in 4.3 s (0.330 / 0.343 m off)` and `stop 2/12 MISSED after 45 s: 2.704 / 2.707 m from it` |
| the AP series: what the new arrangement produced (2026-10-15) | `STATION_MODE=ego`, the 12-stop circle, `logs/missionA-AP1-stations`, `-AP2-stations`, `-AP3-stations` | AP1/AP3: **1 of 12** stops REACHED (stop 1, 0.330 / 0.343 m off, 5 s dwell), stop 2 MISSED after 45 s at **2.704 / 2.707 m**; **C2 min clearance 0.342 / 0.345 m with 0 of 82934 / 83036 samples inside the 0.20 m footprint**; A1 3.97 / 4.10 mm, A2 3.98 / 3.93 mm, A4 29.75 mm (AP3), C1 0.015 / 0.014 m; B4 landed NO; ABORT at t = 81.14 / 81.22 s.  **The blocker is measured**: ego committed exactly ONE spline (`bspline.csv` = 1 row) and never planned waypoint 2, so the adapter had nothing but an expired plan to withhold and the FSM waited out its 60 s.  AP2 armed the same tour and committed no spline at all |
| scoring | `tools/metrics.py`: A1/A2 (LIO vs truth), A3 (delivered-stream continuity, bounds anchored to the acceptance table), A4 (the FCU's own `XKF1` vs truth, clock-aligned, best-fit lag reported), B1-B4 (truth vs the commanded geometry), **C1** final point, **C2** minimum truth-to-pillar-surface distance from the world file | every line prints its denominator; `tools/batch-summary.py` aggregates them |

**THREE RULES THIS STACK PAID FOR**, each a class rather than an incident:

1. **One geometry source.**  `tools/world-info.py` is the only reader of the world file; the sensor, the
   trajectory source, the FSM's geofence, the adapter's clearance gate and the metrics all take their
   geometry from it.  MEASURED: `run-mission.sh` started the sensor WITHOUT `--world`, so it rendered
   `worlds/pillars.yaml` while everything else used `worlds/forest_all.yaml` -- two different forests, and
   Mission B's plans were judged against pillars the aircraft never flew near.
2. **An interface has three properties: name, type and QoS.**  `/cloud_registered` did not exist under that
   name; once it did, the planner dropped every message because a BEST_EFFORT publisher cannot feed a
   RELIABLE subscription.  A topic is not connected until the CONSUMER's own log stops complaining.
3. **A number carries its rate, its frame, its window -- and its DELAY is a number too.**  A3's bound was
   written "per sample at 200 Hz" for a stream arriving at 33.3 Hz; A4 grows with speed because the velocity
   is ~80 ms old, and its gate now scores the residual at the best-fit lag rather than at zero.

## 4.0b The Mission-B chain, node by node (2026-10-15, rev `7a00430`)

The boxes above are the design intent; this is the chain as it is wired for mission B, in the order the
data flows, with the file that owns each hop.  A hop that has not been in the air says so.

1. **LIO -> scan relay.**  `sim/lidar_imu.py` -> `/livox/lidar` (10 Hz, S) + `/livox/imu` (200 Hz, S) ->
   Super-LIO -> `/lio/robo/odom` (200 Hz, L, body pose) and `/lio/odom` (10 Hz, L, WITH velocity).
   `tools/scan_relay.py` takes the sensor's RAW scan plus the LIO's own body pose and publishes
   `/cloud_registered` (L, RELIABLE).  The LIO's own `/lio/cloud_world` is its VOXEL map (0.25 m) and is
   NOT what the planner eats: measured, its closest point to a pillar surface was **-0.244 m**, i.e. a
   voxel centre sits inside the pillar, and the planner's plans then landed 0.306-0.311 m from the TRUE
   surface while keeping a comfortable margin in their own thin map.
2. **ego-planner** (`planner_ws`, `23a8d5a` + `patches/0002..0008`) consumes `/cloud_registered` +
   `/lio/robo/odom` and -- since patch 0008 -- **its own station list**: `$OUT/stations.yaml` is a second
   `--params-file` carrying `fsm/flight_type=2` (PRESET_TARGET), `fsm/waypoint_num`,
   `fsm/waypointN_{x,y,z}`, `fsm/waypoint_advance_m`, `fsm/waypointN_dwell_s`, `fsm/waypoint_loop=false`
   and `fsm/waypoint_start_alt`.  It plans `odom -> waypoint` per stop and advances on a test ON THE
   VEHICLE, so nothing outside can run ahead of the aircraft.
3. **`planning/bspline` -> `planner/ego_adapter.py`.**  The adapter is the only file that includes
   `traj_utils`.  It (a) samples the spline into a `nav_msgs/Path` in L with per-point stamps
   `start_time + t_i`, (b) RE-PROJECTS the path to start at the point nearest the vehicle, (c) clamps z to
   the goal's own altitude, (d) gates the ARENA and the **0.20 m footprint** against the world file's
   analytic cylinders -- publishing a verified prefix when the violation is in the far half, because
   withholding the whole path deadlocks the mission -- and (e) WITHHOLDS an expired (`t_cur >= dur`) or
   degenerate (< 0.10 m of arc left) spline WITH A REASON: counted, printed, and after `--stall-s` of the
   same trajectory raised on `/planner/stalled`.  Both halves have now been seen: the WITHHOLD fired in
   AP1/AP3 (`PATH WITHHELD (1): the spline's clock has run out (t_cur 3.900 of 3.900 s, committed 3.9 s
   ago)`), where the old code published a stub instead -- but the ESCALATION killed the adapter
   (`NameError: name 'Bool' is not defined`, an import local to the setup function) until `7a00430`, so
   `/planner/stalled` has never actually reached the wire in flight.
4. **`/planner/traj` -> `fsm/mission_fsm.py`.**  The consumer runs the plan ON THE PLAN'S OWN CLOCK
   (`t_rel = now - t0 + spline_lead`, no rebasing), streams ArduPilot GUIDED setpoints at 50 Hz, and owns
   the decisions: the geofence (TRUTH-fed -- sim only, not a real-world safety net), the world-contract
   hold (`_hold_is_legal`: the straight segment from the vehicle to the target must clear the 0.20 m
   contract -- and that threshold IS the contract, with no margin above the tracking error, which is the
   review's [L5]) and a BOUNDED, ATTRIBUTED abort: a plan whose clock ran out, with the pull illegal, is
   named at 1 Hz and aborted after `--late-plan-abort-s` = 20 s, instead of the ~490 s silence of AO1.
5. **the station bookkeeping is an OBSERVER.**  `planner/goal_sequencer.py --observe-wp-index /wp_index`
   publishes NO goals and no index of its own: it mirrors ego's `wp_index` (`std_msgs/Int32`, published on
   every change plus a 1 Hz keep-alive, `WP-INDEX: k of N`, -1 = the tour is over) onto `/mission/stop`,
   and still SCORES the stops from truth.  FLOWN (AP1/AP3): the sequencer's log says `goal published 0
   time(s) for it` while it scored `stop 1/12 REACHED in 4.3 s (0.330 / 0.343 m off) -- holding 5.0 s`
   and `stop 2/12 MISSED after 45 s: 2.704 / 2.707 m from it -- moving on`, i.e. it can neither fabricate
   a goal nor walk the stop list on its own any more.  Under PRESET_TARGET the adapter's whole goal path is
   DEAD CODE (`ego_adapter.py:492-584`; the station list is the only goal source) -- left in place, unused.
6. **FCU.**  `bridge/mavlink_impl.py` (the flown path; `BRIDGE=mavros` exists but is unflown) turns the
   FSM's F-frame numbers into `SET_POSITION_TARGET_LOCAL_NED` and the LIO pose into
   `ODOMETRY`/`VISION_POSITION_ESTIMATE`; ArduPilot Copter GUIDED runs its own PSC controller and flies
   it.  No Gazebo anywhere.
7. **the truth tap is the metric layer, never the control layer.**  `/sim/truth/odom` and the tap CSV
   feed the metrics (A1-A4, B1-B4, C1, C2), the window segmentation and the geofence; nothing in the
   control path reads them except that geofence, which is why it is labelled sim-only.

### 4.0c The `traj_server` borrowing boundary (what we took, what we did not, what is still worth taking)

`src/planner/plan_manage/src/traj_server.cpp` is upstream's own consumer, and it is UNUSED here (no PX4,
no `quadrotor_msgs` controller).  It is the reference for one thing only -- HOW A SPLINE IS CONSUMED --
and the boundary was drawn deliberately.

**TAKEN, because it is what the spline deserves:**
* **A new B-spline REPLACES the current one** (`traj_.clear()`), and `receive_traj_` gates all output, so
  nothing is published until a plan exists.
* **`t_cur = (now - start_time_).seconds()`** -- the spline is evaluated ON ITS OWN CLOCK, never rebased to
  the moment the consumer noticed it (`traj_server.cpp:172`; `mission_fsm.py:896-904` cites it by hand).
  MEASURED BEFORE, sortie AI1: rebasing treated the Path as a GEOMETRIC path to be joined from its first
  point, which created the "connector" the adapter then had to gate, and starved the mission whenever that
  straight line crossed a pillar.
* **Past the end: zero the feed-forward.**  Upstream hovers at the spline's last point with vel/acc zero.
  The zeroing is essential here too: MEASURED sortie F2, keeping the last segment's velocity meant the
  aircraft flew on at 2.43 m/s past a stop that had been commanded 4.18 m behind it -- 8 m of flight, out
  of the arena, until the geofence stopped it.  What we do NOT take is the unconditional hover -- our last
  point is often metres away and is not the goal -- so a clock that is out AND a pull that is illegal means
  "hold here", per hop 4 above.
* **Yaw as a slew-limited direction of travel.**  Upstream takes yaw from the look-ahead point and slews at
  `YAW_DOT_MAX_PER_SEC = PI` = **3.14 rad/s** (`traj_server.cpp:73-74`; an earlier note in this project
  said 1 rad/s -- the code is the authority).  Ours uses the plan's own velocity azimuth, freezes the
  heading near a station and slews at `--yaw-rate-max` 120 deg/s = **2.09 rad/s** (the safe direction).
  OPEN MEASURED DEFECT: the DEFAULT is still `--yaw-mode fixed`, and with the nose pinned North on the
  circle **83.4 %** of the moving flight was more than 30 deg off its velocity vector (19878
  airborne-and-moving samples, `docs/metrics/phaseB-yaw-vs-velocity.txt`).

**NOT TAKEN, and why:**
* `quadrotor_msgs::PositionCommand` and the separate controller (`px4ctrl` in the Fast-Drone-250 family):
  ArduPilot Copter GUIDED accepts position/velocity local-NED setpoints and runs its own PSC controller, so
  there is no counterpart to port.  The cost is recorded rather than hidden: nothing sits between a bad
  plan and the motors, and the FSM owns arming, the mode, LAND, the abort and (truth-fed) the geofence.
* The 100 Hz wall timer, the 2 s look-ahead horizon and the `kx/kv` gains: those are properties of that
  controller, not of the interface.

**STILL WORTH TAKING -- the two this project has not done:**
1. **Analytic velocity and acceleration from the spline's own derivative splines.**  Upstream asks the
   B-spline for position, velocity AND acceleration; we sample position only and the FSM differences the
   per-point stamps (`ref_at`, ~20 ms grid), which is exact enough at 1 m/s and the wrong shape at 3 m/s.
2. **A `trajectory_flag` / `trajectory_id`-style signal** -- "this is a new plan" and "I have no plan".
   Freshness here is still "the first point's stamp changed" (`mission_fsm.py:913-914`), so two different
   plans with the same first stamp are indistinguishable, and a withheld plan is a drop rather than a
   statement.  Upstream carries `TRAJECTORY_STATUS_READY` plus `traj_id_` and its controller uses
   `trajectory_id` to detect a new plan; the adapter's `/planner/stalled` is a partial answer to the other
   half, and only after `--stall-s`.

## 4. Topics — rate, content, frame

| topic | type | rate | content | frame | producer -> consumer |
|---|---|---|---|---|---|
| truth tap (UDP, not ROS) | ASCII line | **1200 Hz measured** (the model rate) | `TAP <inst> <t_us> p_ned v_ned q(w,x,y,z body->NED) gyro FRD accel FRD` | SITL native, declared (PLAN #13) | box 1 -> box 2 |
| tap fan-out (UDP, local) | the same lines | 1200 Hz | the same stream, re-sent to 14561 so `sim/lidar_imu.py` can consume it as a second process (only one process can bind the tap's port) | as above | box 2 -> box 2 |
| `/clock` | `rosgraph_msgs/Clock` | 50 Hz requested, **48.0 measured** | sim time (the single time authority) | -- | box 2 -> all |
| `/sim/rtf` | `std_msgs/Float32` | 1 Hz | sim seconds per wall second (**1.000-1.002 measured** with the sensor node running) | -- | box 2 -> box 7 |
| `/livox/lidar` | `sensor_msgs/PointCloud2` | **10 Hz requested, 10.004 measured** | 20 000 points; x,y,z,intensity,time; `time` = per-point offset 0..0.099995 s in SECONDS (what `LID_TYPE::VELO16` reads); stamp = scan start | **S** (`mid360_link`) | box 2 -> box 3 |
| `/livox/imu` | `sensor_msgs/Imu` | **200 Hz requested; 201.8 measured on a full-rate tap, 199.4-200.1 measured in the mission** (the mission's fan-out is decimated to 200 Hz, and the decimation is derived from the rate that arrives, not assumed -- see `STATUS.md` row 37) | gyro rad/s, accel m/s^2, no orientation, in S | **S** | box 2 -> box 3 |
| `/sim/truth/odom` | `nav_msgs/Odometry` | 100 Hz requested, **92.7 measured** (the node also parses 1200 lines/s; the lossless truth channel is the tap CSV) | true body pose/velocity (never used for control) | **W** | box 2 -> box 7 |
| `/sim/world/markers` | `visualization_msgs/MarkerArray` | 1 Hz, transient-local | the loaded world's pillars + arena + the mission geometry (mission A: 46 pillars, 49 markers measured; mission B's `forest_b.yaml` has 64) | **W** | box 2 -> RViz |
| `/clicked_point` | `geometry_msgs/PointStamped` | on click | operator adds or moves a pillar | **W** | RViz -> box 2 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | on change | the mission's next stop, in W -- published by `planner/goal_sequencer.py` ONLY when it is not in observer mode; under `STATION_MODE=ego` NOTHING publishes it (the stations are inside ego) | **W** | box 4 (sequencer) -> box 4 (adapter) |
| `/lio/odom` | `nav_msgs/Odometry` | **10 Hz** | ESKF state at scan rate, with the velocity the FSM needs (its twin `/lio/robo/odom` carries only the pose) | **L** | box 3 -> box 7 |
| `/lio/robo/odom` | `nav_msgs/Odometry` | **200 Hz** (= the IMU rate: published from the IMU callback with the IMU's stamp) | body pose, zero twist | **L** | box 3 -> box 4 (the planner's state and map centre), box 5, recorder |
| `/odom` | `nav_msgs/Odometry` | **200 Hz** | fitted body pose + velocity, child `base_link` | **L** | box 3 -> box 5, RViz |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | **10 Hz** | the sensor's RAW scan registered into L by `tools/scan_relay.py` (the LIO's own `/lio/cloud_world` is a 0.25 m VOXEL map and is NOT used here), publisher QoS **RELIABLE** to match the planner's subscription | **L** | box 3 + box 7 (relay) -> box 4, RViz |
| `/lio/map` | `sensor_msgs/PointCloud2` | 0.2-1 Hz | accumulated map (optional) | **L** | box 3 -> RViz |
| `/lio/status` | `diagnostic_msgs/DiagnosticArray` | 1 Hz | measured scan rate, effective points, degeneracy lambda2/lambda1, map size | -- | box 3 -> box 5 gate, box 7 |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | 200 Hz / static | `world_w -> odom -> base_link -> mid360_link` | -- | box 5 (static pair), box 3 (odom edge) |
| ~~`/planner/goal`~~ | -- | -- | **RETIRED, no publisher**: the goal path is `/goal_pose` (W) -> the adapter -> `/move_base_simple/goal` (L), and under `fsm/flight_type=2` even that path is dead code | -- | -- |
| `/planner/traj` | **`nav_msgs/Path`** | points at >= 20 Hz of trajectory time, republished >= 2 Hz (4.00 Hz median measured), latched (TRANSIENT_LOCAL, depth 1) | pose + yaw per point, each point stamped with the instant the planner means it to be reached (`start_time + t_i`), dense enough to difference for velocity/acceleration | **L** | box 4 (`rectangle_source` for mission A, `ego_adapter` for mission B) -> box 5 |
| `/planner/status` | `std_msgs/String` | 1-10 Hz | replan ok/fail + reason | -- | box 4 -> box 5 |
| `/wp_index` | `std_msgs/Int32` | on change + 1 Hz keep-alive | **ego's own station index** (patch 0008), with a `WP-INDEX: k of N` log line; `-1` = the tour is over | -- | box 4 (ego) -> box 4 (the observer sequencer) |
| `/mission/stop` | `std_msgs/Int32` | on change | the stop the mission is ON: the observer sequencer mirrors `wp_index` here unchanged, so the FSM's interface is the same but the number comes from the planner that is flying; `-1` = sequence complete | -- | box 4 (sequencer, observer) -> box 5 |
| `/planner/stalled` | `std_msgs/Bool` | after the SAME trajectory has been withheld for `--stall-s` | "the planner has no plan" -- the adapter's withhold-with-a-reason, escalated | -- | box 4 -> box 5 / the orchestrator |
| `/fsm/phase` | `std_msgs/String` | on change + 1 Hz | preflight / takeoff / hold / leg1..4 / land / abort | -- | box 5 -> box 7 (window segmentation) |
| setpoint 84 | MAVLink | **20-50 Hz** | sampled position (+ optional velocity/accel/yaw/yaw_rate), `LOCAL_NED` | F, NED on the wire | box 5 -> box 1 |
| command 76 | MAVLink | on demand | arm / takeoff / land / mode | -- | box 5 -> box 1 |
| vision 102 / odometry 331 | MAVLink | **48.2 Hz measured at the FCU** (`VISP`/`VISV` in its own log): box 5 re-samples the newest LIO message in its own 50 Hz loop, so this is not the LIO's output rate and not the recorder's rate | LIO pose + velocity (+attitude) | F, NED on the wire | box 5 -> box 1 |
| telemetry 32 | MAVLink | **20-50 Hz** | FCU local position/velocity (**z DOWN**) | F | box 1 -> box 5 |
| telemetry 31 | MAVLink | **20-50 Hz** | FCU attitude (FRD body -> NED) | F/B | box 1 -> box 5 |
| 0 / 253 | MAVLink | 1-2 Hz / on event | armed, mode, health / text | -- | box 1 -> box 5 |
| optional occupancy grid | `nav_msgs/OccupancyGrid` | 1-5 Hz | 0.1 m cells from `/cloud_registered` | **L** | box 4 (or box 3) -> RViz, fallback planner |

With `BRIDGE=mavros` the MAVLink rows become mavros topics (`/mavros/local_position/pose` 30-50 Hz ENU,
`/mavros/state` 1 Hz, `/mavros/statustext/recv`, `/mavros/estimator_status`, `/mavros/home_position/home`,
`/mavros/time_reference`, setpoints on `/mavros/setpoint_position/local`, vision on
`/mavros/vision_pose/pose_cov`) and **mavros performs the frame conversions**.

### Two disciplines the old pipeline taught, kept here

* **Rates are judged by the publisher's own counters, not by `ros2 topic hz`.**  Measured in the old
  project: two consecutive `topic hz` readings of the same topic differed by more than 3x under load.
  Every rate-sensitive node therefore reports its achieved rate in its own status topic (`/sim/rtf`,
  `/lio/status`), and the metrics read those.
* **No orphan parameter files.**  In the old tree a parameter file existed that no launch path loaded,
  and reasoning from it gave wrong conclusions about which topics were live.  Here every file under
  `sim/`, `lio/`, `planner/` must be referenced by a launch path, checked by grep in review.

### 4b. FCU link rates: what the firmware allows, and how to ask for it

Read out of Copter-4.7.1 on 2026-10-13, because "can it be 50 Hz" has a hard answer per message.

| direction | message | how its rate is set | maximum |
|---|---|---|---|
| FCU -> us | **32** `LOCAL_POSITION_NED` (and `GLOBAL_POSITION_INT`) | parameter **`SR1_POSITION`** (`GCS_MAVLink_Parameters.cpp`, `@Range: 0 50`) -- **default is 3 Hz**, useless for arrival checks | **50 Hz** |
| FCU -> us | **30** `ATTITUDE` (euler), also `SIMSTATE`, `AHRS2`, ... | parameter **`SR1_EXTRA1`** (`@Range: 0 50`), default 10 Hz | **50 Hz** |
| FCU -> us | **31** `ATTITUDE_QUATERNION` | the message is known to the GCS layer (`ap_message.h:20`, `MSG_ATTITUDE_QUATERNION`) but is **not in any `SR*` group**, so it is not streamed by default: request it with **`MAV_CMD_SET_MESSAGE_INTERVAL` (511)**.  Using msg 30 instead and converting euler -> quaternion in the bridge avoids the extra request entirely | request is capped at `loop_period / 0.8`; SITL's 400 Hz loop gives ~3.1 ms, i.e. **up to ~320 Hz**; a faster request is **DENIED** with a statustext |
| us -> FCU | **84** `SET_POSITION_TARGET_LOCAL_NED` | not rate-limited by the FCU; consumed at the control loop rate (SITL 400 Hz).  **Position setpoints do NOT time out** -- `mode_guided.cpp:1194` is explicit that `GUID_TIMEOUT` is "only used for velocity, acceleration, angle control, and angular rates" | 50 Hz is comfortably fine |
| us -> FCU | **82** `SET_ATTITUDE_TARGET` | same, **but this one IS angle/rate control**, so `GUID_TIMEOUT` (default 3 s) applies: if we stop sending, the FCU zeroes the lean angles and climb rate | 50 Hz fine; raise to 100+ Hz if attitude tracking looks coarse |
| us -> FCU | **102** `VISION_POSITION_ESTIMATE` / **331** `ODOMETRY` | **no rate limit in AP_VisualOdom** (the only state it keeps is a health timestamp, `_last_update_ms`); each accepted update is handed to the EKF | 50 Hz fine; set `VISO_DELAY_MS` (0-250, default 10) to our *measured* pipeline latency rather than guessing |

Consequences for the design:

* The FSM streams setpoints at **50 Hz**; the telemetry we depend on is raised to **`SR1_POSITION = 50`** and
  **`SR1_EXTRA1 = 50`** at start-up, and the bridge **verifies the achieved rates** (its own counter)
  rather than trusting the request.
* 50 Hz is the declared maximum of the stream-rate parameters, so 50 Hz is not a compromise -- it is the
  firmware's own ceiling for telemetry.  Above that only the per-message interval command can go, and it
  is bounded by the loop rate.
* Bandwidth: on the local UDP/TCP link this is nothing; on a real radio 50 Hz x several messages is a lot,
  which is one more reason the sim link and the radio link are configured separately.
* Because **position** setpoints have no timeout, the "keep streaming while holding" rule is about
  smoothness and about not being in a velocity/attitude phase when the link hiccups -- not about keeping
  GUIDED alive.

## 5. TF tree

```
world_w  (W: world.yaml geometry, pillar markers)
   |  static, from Calib (yaw + translation, measured once per flight, checked against construction)
odom     (L: the LIO world, also the planner's world)
   |  from /odom at 200 Hz
base_link (B)
   |  static mount: 45 deg nose-down + (0.05, 0, 0.10)
mid360_link (S)
```

`map` is deliberately not used: the LIO has no loop closure or jumps, so one continuous frame suffices.
RViz shows pillars (in `world_w`) and the cloud (in `odom`) together because of that static edge.  The FSM
and the planner never touch W except when the mission is set up -- and that set-up now includes the
**station list**, which `tools/mission-params.py` converts W -> L ONCE at launch (`FRAMES.md` §7.2), so no
goal is converted per goal any more.

### Displaying it in RViz (asked: does anything need to be unified, will the cloud look tilted?)

Nothing needs to be *unified* -- that is what TF is for -- but one frame must be chosen and the tree must
reach it, and two specific things will look wrong if it does not:

1. **RViz Fixed Frame**: set it to **`world_w`**, so the world file's geometry, the pillars, the mission
   square and the cloud all appear in the frame the world is written in.  (Debugging the raw sensor view is
   the other useful choice: set it to `mid360_link`.)
2. **The raw scan is in S and S is pitched 45 deg.**  `/livox/lidar` is the housing frame, so displaying it
   without the `base_link -> mid360_link` static transform draws a cloud rotated 45 deg about the body's
   y axis -- the classic "the point cloud is tilted".  With the transform, it lands correctly.
3. **Pillars are in W and `/cloud_registered` is in L, and L is W rotated by a constant** (-90 deg with the
   North-facing spawn of `FRAMES.md` 0.3) **plus the ~0.3 m origin offset.**  Without the static
   `world_w -> odom` edge the two appear rotated by 90 deg relative to each other and offset -- again
   exactly the artifact that looks like a broken cloud.  With it, both are correct in either Fixed Frame.
4. **One time base.**  TF lookups happen at the message's stamp, so if RViz and the publishers disagree
   about the clock (missing `use_sim_time`, missing `/clock`) the cloud flickers or disappears even though
   the frames are right.  `/clock` from box 2 is the single authority.

## 6. The transport interface (bidirectional, normalised)

Both implementations return the SAME numbers in the SAME frame: **`F`, ENU-labelled, metres, z up**,
quaternion = body(FLU) -> F.  The FCU's wire convention is reached inside the implementation, and each
implementation crosses exactly once.

```python
class Transport(Protocol):                       # both directions
    # outbound
    def arm(self, force=False) -> bool: ...
    def set_mode(self, mode: str) -> bool: ...
    def takeoff(self, alt_m: float) -> bool: ...
    def send_position(self, p_f, yaw_rad=None) -> None: ...      # ENU labels, z up
    def send_velocity(self, v_f, yaw_rate=0.0) -> None: ...
    def send_external_nav(self, stamp, p_f, q_f, v_f) -> None:   # LIO -> FCU
    # inbound
    def state(self) -> VehicleState: ...          # connected / armed / mode / statustext
    def local_pose(self) -> Pose: ...             # msg 31+32, normalised to ENU / z up
    def local_velocity(self) -> Vec3: ...
```

| direction | `mavlink_impl` (pymavlink) | `mavros_impl` |
|---|---|---|
| setpoints out | `f_to_ned()` then msg 84 (z becomes DOWN) | publish `/mavros/setpoint_position/local` (mavros converts) |
| external nav out | ENU->NED + FLU->FRD + origin, then msg 102/331 | publish `/mavros/vision_pose/pose_cov` (mavros converts) |
| pose in | msg 32 + 31 -> `ned_to_f()` + `frd_to_flu()` | subscribe `/mavros/local_position/pose` (already ENU/FLU) |
| state in | `HEARTBEAT` + `STATUSTEXT` | `/mavros/state` + `/mavros/statustext/recv` |

**The trap this interface exists to kill:** `LOCAL_POSITION_NED.z` is DOWN, `/mavros/...pose.z` is UP.
"Hover at 2.5 m" is `-2.5` on one path and `+2.5` on the other, so an arrival check written against the
raw message is correct under one bridge and silently broken under the other.  The tests assert that BOTH
bridges report `+2.5`.

## 7. Rates, latency, compute (measured where noted)

| stage | rate | cost | note |
|---|---|---|---|
| truth tap | **1200 Hz measured** (the model rate, `SIM_RATE_HZ`) | negligible | one UDP line per model step |
| lidar sim | 10 Hz x 20 000 rays | **9.8 ms/scan** (46 pillars) | 103 Hz capable; CPU is enough, GPU not needed |
| IMU sim | 200 Hz | negligible | generated from the tap |
| LIO | 10 Hz in, 200 Hz out | ~1 core | dominant cost |
| planner | 10-50 Hz replan | ~1-2 cores | ESDF build dominates |
| FSM | 50-100 Hz | negligible | arrival checks + setpoint streaming |
| FCU | 400 Hz loop | SITL built-in model | no Gazebo anywhere |
| end-to-end | -- | target < 100 ms | scan 100 ms + LIO + planner horizon |

## 8. Deliberate differences from the old Gazebo pipeline

| old | here | why |
|---|---|---|
| Gazebo `gpu_lidar` at 500x40 rays, 2 cm range quantisation, physics-step IMU defect | our analytic ray caster, exact, real MID-360 pattern, our own noise model | the old sensor was the source of a phantom IMU step and of a starved matcher |
| `gz_livox_bridge` converting to `livox_ros_driver2/CustomMsg` | one `PointCloud2` in the velodyne layout, `lidar_type: 3` | the CustomMsg path in this fork **drops the per-point time**, i.e. no motion compensation |
| airframe bounding box removes self-hits, `/sensors/lidar/body_removed_ratio` | no airframe geometry at all -> no self-hits | the world is cylinders + planes; if the airframe is ever modelled, a body-exclusion step and its ratio topic come back |
| `/uav/*` normalisation layer, `/sensors/*` relay, orphan parameter files | one interface per function (the transport), no orphans | the old layer ended up referenced only by an orphan config |
| three terminals, MAVProxy router, `/uas1/mavlink_*` | one MAVLink endpoint to SITL plus the truth tap | fewer moving parts; the FSM owns the link |
| `/fastlio_occupancy_grid` as the planner input | ego-planner builds its own ESDF from `/cloud_registered`; a grid is optional and only for RViz/fallback | one map, one consumer |

## 9. Safety gates (owned by `fsm/`)

1. **Odometry sanity** before feeding the FCU: rate >= 5 Hz, position jump < 0.5 m between scans, speed
   < 3 m/s, `/lio/status` degeneracy above threshold.  On violation: stop ExternalNav, hold; if it
   persists, LAND.
2. **Geofence**: |x|, |y| inside the arena minus the vehicle radius; z between 0.3 m and ceiling - 0.3 m.
   On violation: abort the mission, hold, LAND.
3. **Setpoint continuity**: stream at 50 Hz.  Note the correction from the firmware source: **position**
   setpoints have no timeout at all, so this is about smoothness.  The 3 s `GUID_TIMEOUT` applies to
   velocity, acceleration, angle and rate control -- which is what the planner path uses, so there the rule
   is a hard requirement, not a preference.
4. **Planner failure**: if `/planner/status` reports failure for > 1 s while moving, hold; if it
   persists, fly straight to the goal slowly.  The adapter's own withhold (an expired or degenerate
   spline) raises `/planner/stalled` once the SAME trajectory has been withheld for `--stall-s` (2 s), so
   "the planner has no plan" is a signal rather than an absence.  MEASURED AP1/AP3: that escalation was
   fatal the first time it was reached (`NameError: name 'Bool' is not defined` -- the class was imported
   inside the setup function, so the adapter DIED while withholding and `/planner/stalled` never reached
   the wire); fixed in `7a00430` and not yet re-flown.
5. **The plan's clock is not a target** (added 2026-10-15).  When the plan's clock has run out and the
   straight segment to its last point does not clear the 0.20 m contract, the consumer HOLDS where it is,
   names the mechanism at 1 Hz with the clearance number, and ABORTS after `--late-plan-abort-s` (20 s).
   MEASURED AO1: without the diagnostic and the bound, the FSM sat in exactly this branch for ~490 s
   printing nothing while the sequencer walked the stop list on its own timeout.
6. **No GPS, no compass**: arming depends on a healthy EKF (origin set, vision healthy).  A transiently
   invalid FCU state is tolerated for 3 s -- the old project lost 4 of 8 flights to one bad sample.
