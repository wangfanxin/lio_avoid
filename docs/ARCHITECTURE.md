# ARCHITECTURE — boxes, links, topics, frames, interfaces, safety

One of the project's four living documents (budget in `PLAN.md` §7).  Everything here is a contract: rates,
frames and interfaces come from measurements or from the firmware source, and each conversion has a test
named in `FRAMES.md` §5.

**No Gazebo anywhere.**  The lidar is computed by our own analytic ray caster, the IMU is generated from
the simulator's true state, and RViz is used for display and for editing the world.  Where the old
Gazebo-based pipeline had `/mid360/*` (Gazebo Transport), `gz_livox_bridge`, `/sensors/*`, `/uav/*` and
orphan parameter files, this pipeline has exactly one interface per function and no orphans -- a parameter
file that nothing loads is a bug (verifiable by grep).

**CORRECTED 2026-10-16 (mission B): a planner is in the tree again, and it is the one flying.**  The
sentence that stood here -- "No planner anywhere either, since 2026-10-15.  An ego-planner was built
behind the trajectory interface of `PLAN.md` §4.1, flown, and then dropped by the owner; its files ... left
the tree" -- was true when it was written and is false now: `src/planner/` holds `rectangle_source.py`
(method 1, the frozen mission A), and `ego_source.py`, `ego_contract_shim.py`, `ego_on_lio.launch.py`,
`cloud_relay.py`, `fcu_pose_feed.py` (method 2 / Route B, the ego-planner behind the same `/plan/cmd`
contract, flying since round 76 and recorded in `STATUS.md` rows 123-139).  Everything below about the
BOXES still holds: the interface, the bridge and the supervisor did NOT change between the two methods,
which is exactly what `src/tools/test-planner-swap.py` guards.

---

## 1. The chain as built

```
┌══════════════════════════════════════════════════════════════════════════════════════════┐
║ BOX 1   SITL                     ArduPilot Copter-4.7.1, built-in quad dynamics,          ║
║         owns: the true state, the FCU's local NED frame, the flight modes, the EKF        ║
║   ┌──────────────┐   ┌───────────────┐   ┌────────────────────────────────────────────┐  ║
║   │ quad model   │──►│ built-in IMU  │──►│ EKF3  (EK3_SRC1_POSXY/POSZ/VELXY/VELZ/YAW=6)│  ║
║   │ (its physics)│   │ GPS/baro/comp │   │ GUIDED (position [+velocity] setpoints)     │  ║
║   └──────┬───────┘   └───────────────┘   └───────────────────┬────────────────────────┘  ║
║          │ true state                                        │                            ║
║          └──────────► TRUTH TAP (sim-only, env-gated) ───────┼──► UDP 1200 Hz             ║
╚══════════════════════════════════════════════════════════════╪═════════════════════════════╝
        ▲ MAVLink 5760/udp: setpoints 84 @50 Hz, commands 76     │ truth: p/v NED, q FRD,
        │                    telemetry 31/32 @50 Hz, 0/253        │ gyro FRD, accel FRD
        │                                                         ▼
┌═══════╪═════════════════════════════════════════════════════════════════════════════════════┐
║ BOX 2 │  src/sim/           the world, the sensor, the clock, the truth receiver                 ║
║   ┌───┴──────────┐   ┌──────────────────┐   ┌─────────────────┐                          ║
║   │ truth_rx     │──►│ lidar_imu        │   │ clock           │                          ║
║   │ UDP -> ROS   │   │ raycast + noise  │   │ /clock 48 Hz    │                          ║
║   │ 1200 Hz      │   │ IMU model        │   │ /sim/rtf 1 Hz   │                          ║
║   └──────────────┘   └────────┬─────────┘   └─────────────────┘                          ║
║   $WORLD (ONE file) ──────────┘  geometry, mount 45 deg, mission; the SENSOR gets --world  ║
║   too, so generation and judgement cannot disagree                                         ║
╚════════════════════════════════╪══════════════════════════════════════════════════════════╝
     /livox/lidar 10 Hz (S)      │      /livox/imu 200 Hz (S)
     /sim/truth/odom 93 Hz (W)   │      (metrics only -- nothing in the control path reads it,
                ┌────────────────┴───    except the sim-only geofence gate)
                ▼
┌═══════════════════════════════════════════════════════════════════════════════════════════┐
║ BOX 3   lio/   ONE process: Super-LIO, built clean from upstream `f89f48d`                ║
║   in : /livox/lidar 10 Hz (S), /livox/imu 200 Hz (S)                                      ║
║   out: /lio/robo/odom 200 Hz (L, body pose, ZERO twist)                                   ║
║        /lio/odom      10 Hz (L, the same state WITH velocity)                             ║
║        /lio/cloud_world 10 Hz (L) = THE CURRENT FRAME, undistorted and transformed to the world frame       ║
║                                    (CORRECTED 2026-10-16: this line said "its VOXEL map (0.25 m), NOT a raw   ║
║                                     scan", and it was wrong.  `livox_360.yaml:84` sets `lio.output.map: true` ║
║                                     -> `g_visual_map` (`ROSWrapper.cpp:168`) -> the published cloud is       ║
║                                     `scan_undistort_full_` (`super_lio.cpp:570-579`), the full per-frame     ║
║                                     scan; the 0.2 m `voxel_fliter_size` is the MATCHER's `ds_undistort_`      ║
║                                     (the `else` branch) and `ds_size` 0.15 is the internal map, neither of    ║
║                                     which ego ever sees.  Measured consequence: the map's extra obstacle       ║
║                                     thickness is fully explained by ego's own cell + cube inflation --          ║
║                                     predicted outer edge 0.540 m against a measured 0.540 m                     ║
║                                     (`docs/metrics/inflation-20261016.txt`).                                    ║
║        /lio/status 1 Hz, /tf (odom -> base_link)                                          ║
╚═══════════════╪═══════════════════════════════════════════════════════════════════════════╝
                │ pose + velocity (L)
                ▼
┌═══════════════════════════════════════════════════════════════════════════════════════════┐
║ BOX 4   src/planner/rectangle_source.py -- THE PLAN **AND ITS EXECUTION**, one box, no other    ║
║   in : worlds/pillars.yaml through src/tools/world-info.py (the one geometry reader)           ║
║        --calib-file (_stage/calib.json): the SAME measured W<-L the bridge uses            ║
║        /lio/robo/odom (L) + /lio/odom (its velocity)                                       ║
║             <- THE ARRIVAL GATE, THE SETTLE AND THE DWELL: its own waypoint sequencer      ║
║                (`src/planner/cmd_stream.py::TourPacer`), ESTIMATE-ONLY.  The truth tap is not  ║
║                subscribed here, because a real planner has none.                           ║
║   out: /plan/cmd -- quadrotor_msgs/PositionCommand in L, on its OWN 100 Hz wall timer       ║
║        position + velocity + acceleration + yaw + yaw_dot + trajectory_id + trajectory_flag ║
║        the DWELL is CONTENT (position = the corner, velocity = 0); trajectory_id increments ║
║        once per waypoint; trajectory_flag = COMPLETED is the end of the tour and the stream ║
║        KEEPS PUBLISHING (so "finished" and "stopped talking" are different facts)           ║
║   owns: the waypoints, arrival, settle, dwell, leg timeouts, yaw, the command stream        ║
║   does NOT own: modes, arm/takeoff/LAND, the geofence, the abort, the phase log, the FCU    ║
╚═══════════════════════════════════════════════════════════════════════════════════════════╝
                │ /plan/cmd (L)                                          [F1: this arrow replaced
                ▼                                                         the nav_msgs/Path one]
┌═══════════════════════════════════════════════════════════════════════════════════════════┐
║ BOX 5   src/supervisor/supervisor.py         the supervisor's seat, frames, FCU communication   ║
║         (CORRECTED 2026-10-16: this read `fsm/mission_fsm.py`, a path that does not exist)      ║
║   in : /plan/cmd (L, latched)         <- src/bridge/bridge_node.py::CmdRelay consumes it and    ║
║                                          writes the NEWEST sample to message 84 at 50 Hz    ║
║        /lio/robo/odom 200 Hz (L)      /lio/odom (the velocity)                             ║
║        /sim/truth/odom (W)            <- the geofence gate ONLY (sim-only safety)          ║
║        FCU state/pose over MAVLink    [bridge]                                             ║
║   out: setpoint 84 @50 Hz             [bridge: send_cmd, L -> LOCAL_NED, z becomes DOWN]    ║
║        external nav 331 (+102) @48 Hz [bridge: send_external_nav_l, LIO -> EKF]             ║
║        /fsm/phase (String) on change                                                       ║
║   owns: preflight, takeoff, the truth-fed geofence, LAND, and a BOUNDED abort that NAMES    ║
║         the mechanism instead of going silent -- INCLUDING "the plan box stopped talking"   ║
║   does NOT own: the trajectory, the arrival, the settle, the dwell (box 4), the estimate    ║
║         (box 3), the physics (box 1)                                                       ║
║   contains NO frame arithmetic any more: every conversion is inside src/bridge/ (F1)            ║
╚═══════════════════════════════════════════════════════════════════════════════════════════╝

┌───────────────────────────────┐   ┌──────────────────────────────────────────────────────┐
│ BOX 6   RViz                  │   │ BOX 7   src/tools/                                       │
│  shows: /sim/world/markers(W) │   │  recorders: every stream + the truth -> logs/<run>/   │
│         /lio/cloud_world (L)  │   │  metrics.py: A1-A4 / B1-B4 / C1 / C2, offline         │
│         /plan/cmd (L)         │   │  plot-run.py, dwell-audit.py, sortie-score.py         │
│         /tf, /odom, /fsm/phase│   │  frozen-check.py, docs-budget.py: the two rules,      │
│  frame: world_w (mission_a.rviz, FROZEN) │        executable                                    │
└───────────────────────────────┘   └──────────────────────────────────────────────────────┘
```

Frame shorthand: **W** world ENU (the file, origin = pad) · **L** LIO world (z up, origin = LIO init) ·
**F** FCU local (NED on the wire) · **B** body FLU · **S** MID-360 housing.  L and F are the same frame up
to axis labels; W differs from L by a known constant rotation + offset.  `FRAMES.md` owns the details.

## 2. Where the lidar and the IMU come from

**They are generated by us.**  With no Gazebo there is no sensor plugin, so `src/sim/` synthesises both from
two inputs: the **true vehicle state** (SITL's built-in quad model, read through the truth tap) and the
**world geometry** (`worlds/pillars.yaml`).

```
BOX 1 SITL: built-in quad model
        true state at each model step: p, v, q (body->NED), body rates w, specific force a
                 │
                 │ truth tap (sim-only, env-gated, 67 lines in ONE file, libraries/SITL/)
                 │ UDP 1200 Hz (the model rate), ASCII, frame declared in the header: NED / FRD
                 ▼
BOX 2 src/sim/truth_rx ──► the true pose at the instant a scan starts (interpolated)
                 │
      ┌──────────┴────────────────────────────────────────────────┐
      ▼                                                           ▼
 LIDAR  (src/sim/lidar_imu)                                     IMU  (src/sim/lidar_imu)
   rays = the real MID-360 pattern, 20 000 rays per 100 ms     gyro  = w_true + bias + noise
          (scan_mode/mid360.csv, non-repetitive, 200 kHz)      accel = a_true + bias + noise
   hit  = closed-form intersection with cylinders /                (a_true is the specific force,
          floor / ceiling / walls                                  so gravity is already in it)
   noise = range sigma 0.02 m, blind 0.10 m, max 40 m          rate  = 200 Hz, decimation DERIVED
   rate  = 10 Hz, per-point time offsets kept                         from the rate that arrives
   frame = S, mount applied to the ray directions              frame = S (the housing)
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
Gazebo chain the FCU and the LIO consumed the *same* IMU: the Gazebo model's sensor fed both the ArduPilot
plugin and the LIVOX bridge.  Here there are two: SITL's built-in model generates the FCU's IMU
internally, and `src/sim/` generates the LIO's.  They derive from the same true motion, so both estimators see
consistent dynamics, but the noise realisations differ.  If that ever matters -- it would appear as an
attitude mismatch between `XKF1` and the LIO -- the fix is to make our IMU the single sensor and feed it to
SITL as well, which needs the external-sensor path (SIM_JSON) instead of the built-in model.

## 3. Per-box cards (what each box subscribes, publishes, owns)

| box | subscribes (rate, frame) | publishes (rate, frame) | owns |
|---|---|---|---|
| 1 SITL | MAVLink: setpoints 84, commands 76, external nav 102/331 | MAVLink: 31, 32, 0, 253; truth tap UDP **1200 Hz measured** | the true state, the EKF, the modes, the FCU local frame |
| 2 `src/sim/` | truth UDP 1200 Hz; `worlds/*.yaml` | `/livox/lidar` 10 Hz (S), `/livox/imu` 200 Hz (S), `/clock` 48 Hz, `/sim/rtf` 1 Hz, `/sim/truth/odom` 93 Hz (W), `/sim/world/markers` 1 Hz (W) | the world, the sensor models, the clock authority |
| 3 `lio/` | `/livox/lidar`, `/livox/imu` | `/lio/robo/odom` 200 Hz (pose only), `/lio/odom` 10 Hz (with velocity), `/lio/cloud_world` 10 Hz (the VOXEL map), `/lio/status`, `/tf` | the LIO world frame L and the odometry |
| 4 `src/planner/` | `worlds/*.yaml` (through `src/tools/world-info.py`), `--calib-file`, `/lio/robo/odom` + `/lio/odom` (its own arrival/settle gate) | **`/plan/cmd`** (`quadrotor_msgs/PositionCommand`, L, own 100 Hz timer) | the plan **and its execution**: the waypoints, the arrival gate, the settle, the dwell, the leg timeouts and the COMPLETED statement |
| 5 `fsm/`+`src/bridge/` | `/plan/cmd` (the relay's input), `/lio/robo/odom`, `/lio/odom`, `/sim/truth/odom` (geofence only), the FCU's pose/state | setpoints 84, external nav 331/102, `/fsm/phase` | preflight, take-off, the safety gates, the NAMED abort, LAND -- and every frame conversion (the FSM has none) |
| 6 RViz | everything visual | nothing | display only (`rviz/mission_a.rviz` is frozen) |
| 7 `src/tools/` | everything (recorders) and every recording (offline) | logs, tables, figures | the evidence |

## 4. The chain AS BUILT — measured, 2026-10-15

Every row is a measured number from a run directory, and the row says where.

| stage | what really happens | the number, and where it came from |
|---|---|---|
| truth | ArduPilot SITL (Copter-4.7.1 + `src/patches/apm-0001-sitl-truth-tap.patch`) steps at 1200 Hz and emits `TAP` lines on UDP | **1200.5 Hz**, `logs/s3-parked-*`; against the firmware's own SIM2 record: position max **0.0004 mm**, clock bias +0.0000 ms, drift +0.06 ppm |
| sensor | `src/sim/lidar_imu.py` renders `/livox/lidar` (20 000 points, 10 Hz, per-point times) and `/livox/imu` | **199.4-200.1 Hz IMU**, the decimation derived from the rate that ARRIVES, `logs/missionA-imu200a` |
| LIO | clean upstream Super-LIO `f89f48d`: `/lio/robo/odom` (200 Hz, body pose, L, zero twist), `/lio/odom` (10 Hz, with velocity), `/lio/cloud_world` (the 0.25 m voxel map) | 200.1 Hz delivered, `logs/missionA-stepB10-*` |
| FCU feed | the FSM re-samples the newest LIO pose in its own 50 Hz loop and streams ODOMETRY (331) in NED/FRD | the FCU's own log: **VISP/VISV at 48.2 Hz**, `src/tools/bin-extnav-rate.py` |
| EKF | `EK3_SRC1_POSXY/POSZ/VELXY/VELZ/YAW = 6` (external nav), no GPS, no compass; every measurement and process noise set to the MEASURED value in `params/ekf_trust_lio.parm`, with `FS_EKF_THRESH` raised in proportion (it is a RATIO) | `src/tools/bin-params.py` on the flight BIN |
| trajectory | **since F1 the PLAN box is also the EXECUTOR.**  `src/planner/rectangle_source.py` converts the world file's square W -> L with the run's measured calibration and publishes the tour as a **command stream**: five waypoints `[c1,c2,c3,c4,c1]` on `/plan/cmd` at 100 Hz, on its own wall timer.  `src/planner/cmd_stream.py::TourPacer` decides arrival (`--arrive-m` 0.06 m), settle (`--settle-speed` 0.08 m/s, capped at 3.0 s), the dwell (`--hold-s`) and the leg timeout (20 s) from **its own odometry only** -- `/lio/robo/odom`, converted L -> W with the same calibration | W->L->W round trip printed in `traj.log`, worst **0.000000 mm**; the stream's own clock measured on the bench at **99.94 Hz** against the 100 Hz it declares (the S3 shadow declared 100 and emitted 4); the plan's clearance, recorded in W: **0.6000 m** (the world's designed worst case) |
| mission | **the route is the PLAN box's and the phases come off the stream**: `src/bridge/bridge_node.py::CmdRelay` writes the newest sample to MAVLink 84 at 50 Hz, and `fsm/mission_fsm.py` turns each new `trajectory_id` into a `leg` phase -- the same five names, count and boundaries the deleted corner walk produced.  The FSM keeps preflight, take-off, the truth-fed geofence, LAND and the NAMED abort | A1 4.65-4.90 mm, A4 18.6-21.1 mm, **10 of 10 flights**, `docs/metrics/stepB-rectangle-20261013.txt`; the corner hover before F1: **4 of 4 corners >= 5.0 s** and **5 of 5 B2 legs** on `logs/missionA-RECT7-hold5` (`STATUS.md` row 43); the hover under F1: see `STATUS.md` row 52 |
| last flight with no ego node (RECT1-noego) | the whole pipeline with zero planner processes | A1 4.40 mm, A2 5.96 mm, A4 20.72 mm, **C2 min 0.547 m with 0 of 28425 samples inside the 0.20 m footprint**, C1 0.022 m, B4 landed |
| scoring | `src/tools/metrics.py`: A1/A2 (LIO vs truth), A3 (delivered-stream continuity, bounds anchored to the acceptance table), A4 (the FCU's own `XKF1` vs truth, clock-aligned, best-fit lag reported), B1-B4 (truth vs the commanded geometry), **C1** final point, **C2** minimum truth-to-pillar-surface distance from the world file | every line prints its denominator; `src/tools/batch-summary.py` aggregates them; `src/tools/dwell-audit.py` measures the corner dwell from the tap |

**THREE RULES THIS STACK PAID FOR**, each a class rather than an incident:

1. **One geometry source.**  `src/tools/world-info.py` is the only reader of the world file; the sensor, the
   trajectory source, the FSM's geofence and the metrics all take their geometry from it.  MEASURED:
   `run-mission.sh` once started the sensor WITHOUT `--world`, so it rendered `worlds/pillars.yaml` while
   everything else used another forest -- two worlds, and the plans were judged against pillars the
   aircraft never flew near.  The same rule now also covers the pillar CLEARANCE itself: one definition,
   `src/tools/world-info.py::PillarWorld`, imported by everything that prints a distance.
2. **An interface has three properties: name, type and QoS** -- and a fourth one here: **one producer.**
   A latched, RELIABLE stream is the whole contract; the FSM does not know whether it comes from a file, a
   mission script or a planner, which is why dropping the planner cost the FSM nothing.  **Since F1 the
   stream is `/plan/cmd` (`quadrotor_msgs/PositionCommand`, ego's own type), NOT a `nav_msgs/Path`**: the
   plan box's output is a COMMAND, and keeping a second interface "for compatibility" is how the adapter
   comes back (the old `src/planner/ego_adapter.py` existed only to translate ego's native output into a Path).
   The price paid in the same change: `src/tools/record-traj.py` had to move with it, or every run would have
   silently lost its plan-clearance column.
3. **A number carries its rate, its frame, its window -- and its DELAY is a number too.**  A3's bound was
   written "per sample at 200 Hz" for a stream arriving at 33.3 Hz; A4 grows with speed because the
   velocity is ~80 ms old, and its gate now scores the residual at the best-fit lag rather than at zero.

**AND ONE MORE, ADDED 2026-10-15: a producer's own log is not evidence of what the aircraft did.**  The
corner-hold producer logged three 5 s holds while the truth track spent them somewhere else entirely
(`src/tools/dwell-audit.py`, measured on `tap.csv`).  Every claim about the flight is measured from the truth
tap; the producer's log says only what the producer intended.

## 5. Topics — rate, content, frame

| topic | type | rate | content | frame | producer -> consumer |
|---|---|---|---|---|---|
| truth tap (UDP, not ROS) | ASCII line | **1200 Hz measured** (the model rate) | `TAP <inst> <t_us> p_ned v_ned q(w,x,y,z body->NED) gyro FRD accel FRD` | SITL native, declared | box 1 -> box 2 |
| tap fan-out (UDP, local) | the same lines | 1200 Hz, decimated to 200 for the sensor | so `src/sim/lidar_imu.py` can consume it as a second process (only one process may bind the tap's port) | as above | box 2 -> box 2 |
| `/clock` | `rosgraph_msgs/Clock` | 50 Hz requested, **48.0 measured** | sim time (the single time authority) | -- | box 2 -> all |
| `/sim/rtf` | `std_msgs/Float32` | 1 Hz | sim seconds per wall second (**1.000-1.002 measured** with the sensor node running) | -- | box 2 -> box 7 |
| `/livox/lidar` | `sensor_msgs/PointCloud2` | **10 Hz requested, 10.004 measured** | 20 000 points; x,y,z,intensity,time; `time` = per-point offset 0..0.099995 s in SECONDS; stamp = scan start | **S** (`mid360_link`) | box 2 -> box 3 |
| `/livox/imu` | `sensor_msgs/Imu` | **200 Hz requested; 201.8 measured at full tap rate, 199.4-200.1 in the mission** | gyro rad/s, accel m/s^2, no orientation, in S | **S** | box 2 -> box 3 |
| `/sim/truth/odom` | `nav_msgs/Odometry` | 100 Hz requested, **92.7 measured** | true body pose/velocity.  **Never used for control**: only the geofence gate and the metrics read it | **W** | box 2 -> box 5, box 7 |
| `/sim/world/markers` | `visualization_msgs/MarkerArray` | 1 Hz, transient-local | the loaded world's pillars + arena + mission geometry (mission A: 46 pillars, 49 markers) | **W** | box 2 -> RViz |
| `/lio/odom` | `nav_msgs/Odometry` | **10 Hz** | ESKF state at scan rate, WITH the velocity the FSM needs | **L** | box 3 -> box 5, box 7 |
| `/lio/robo/odom` | `nav_msgs/Odometry` | **200 Hz** (= the IMU rate) | body pose, **zero twist** (measured: Super-LIO fills only the pose) | **L** | box 3 -> box 5, box 7 |
| `/lio/cloud_world` | `sensor_msgs/PointCloud2` | 10 Hz | the LIO's own VOXEL map (0.25 m).  Nothing in the flight path consumes it; it is recorded so a clearance number can be checked against the map the trajectory was built from | **L** | box 3 -> box 7, RViz |
| `/lio/status` | `diagnostic_msgs/DiagnosticArray` | 1 Hz | measured scan rate, effective points, degeneracy lambda2/lambda1, map size | -- | box 3 -> box 5 gate, box 7 |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | 200 Hz / static | `world_w -> odom -> base_link -> mid360_link` | -- | box 5 (static pair), box 3 (odom edge) |
| `/plan/cmd` | **`quadrotor_msgs/PositionCommand`** | **100 Hz published, 50 Hz consumed** (its own wall timer; latched TRANSIENT_LOCAL, depth 5) | position/velocity/acceleration + yaw + yaw_dot, the frame in `header.frame_id`, `trajectory_id` = ONE PER WAYPOINT, `trajectory_flag` = READY while flying and **COMPLETED** at the end of the tour.  The dwell is content: position = the corner, velocity = 0.  Since F1 this is the ONLY plan interface -- the `nav_msgs/Path` that used to be here is deleted | **L** (`lio_world`; ego's name for the same frame, `world`, is accepted by configuration) | box 4 -> box 5 (src/bridge/bridge_node.py) |
| `/fsm/phase` | `std_msgs/String` | on change + 1 Hz | preflight / takeoff / hold / leg1..N / land / abort / done (N = 5 for the square: the entry pad -> c1 plus the four square legs).  The `leg` phases come from the stream's `trajectory_id`, and `src/tools/metrics.py` windows A1-A3/B1-B3/B4/C1 with them | -- | box 5 -> box 7 (window segmentation) |
| setpoint 84 | MAVLink | **50 Hz** | sampled position (+ optional velocity/accel/yaw/yaw_rate, `--setpoint-mode`), `LOCAL_NED`.  One entry point since F1: `bridge.send_cmd(cmd_l, calib)`, which does the L -> wire conversion itself | F, NED on the wire | box 5 (relay) -> box 1 |
| command 76 | MAVLink | on demand | arm / takeoff / land / mode / simple accel calibration | -- | box 5 -> box 1 |
| vision 102 / odometry 331 | MAVLink | **48.2 Hz measured at the FCU** (`VISP`/`VISV` in its own log) | LIO pose + velocity (+attitude), through `bridge.send_external_nav_l` since F1 | F, NED on the wire with the FRD *labels* ArduPilot requires | box 5 -> box 1 |
| telemetry 32 / 31 | MAVLink | **50 Hz** | FCU local position/velocity (**z DOWN**) / attitude (FRD body -> NED) | F / B | box 1 -> box 5 |
| 0 / 253 | MAVLink | 1-2 Hz / on event | armed, mode, health / text | -- | box 1 -> box 5 |
| `/planner/stalled` | `std_msgs/Bool` | -- | **KEPT IN F1 ON PURPOSE**: a trajectory source saying "I have stopped producing plans".  The FSM still subscribes and still turns it into a named abort, but the dropped planner chain was its only publisher, so today nothing raises it.  Its successor -- `CmdRelay.stalled_reason`, the command stream's own liveness -- was added beside it in the same step and names a silent plan box | -- | (nobody) -> box 5 |

With `BRIDGE=mavros` the MAVLink rows become mavros topics (`/mavros/local_position/pose` 30-50 Hz ENU,
`/mavros/state` 1 Hz, setpoints on `/mavros/setpoint_position/local`, vision on
`/mavros/vision_pose/pose_cov`) and **mavros performs the frame conversions**.  `BRIDGE=mavlink`
(pymavlink) is the flown path.  **CORRECTED 2026-10-15: there is no `src/bridge/mavros_impl.py`.**  `ls src/bridge/`
is `base.py` and `mavlink_impl.py`, and `git log` shows no deletion -- the file was never written, while
this document, `src/bridge/base.py`'s own header and `STATUS.md` all described it as "written, unflown" and
`fsm/mission_fsm.py` imported it as the non-mavlink branch.  So `BRIDGE=mavros` raises
`ModuleNotFoundError` at the first call, and everything above describes the CONTRACT that implementation
would have to meet, not a file.  The `mavros` rows below are kept for that reason, in the conditional.

### Two disciplines the old pipeline taught, kept here

* **Rates are judged by the publisher's own counters, not by `ros2 topic hz`.**  Measured in the old
  project: two consecutive `topic hz` readings of the same topic differed by more than 3x under load.
  Every rate-sensitive node reports its achieved rate in its own status topic, and the metrics read those.
* **No orphan parameter files.**  In the old tree a parameter file existed that no launch path loaded, and
  reasoning from it gave wrong conclusions about which topics were live.  Every file under `params/`,
  `src/sim/`, `lio/`, `src/planner/` must be referenced by a launch path, checked by grep in review.

## 6. FCU link rates: what the firmware allows, and how to ask for it

Read out of Copter-4.7.1 on 2026-10-13, because "can it be 50 Hz" has a hard answer per message.

| direction | message | how its rate is set | maximum |
|---|---|---|---|
| FCU -> us | **32** `LOCAL_POSITION_NED` (and `GLOBAL_POSITION_INT`) | parameter **`SR1_POSITION`** (`@Range: 0 50`) -- **default is 3 Hz**, useless for arrival checks | **50 Hz** |
| FCU -> us | **30** `ATTITUDE` (euler), also `SIMSTATE`, `AHRS2` | parameter **`SR1_EXTRA1`** (`@Range: 0 50`), default 10 Hz | **50 Hz** |
| FCU -> us | **31** `ATTITUDE_QUATERNION` | not in any `SR*` group: request it with **`MAV_CMD_SET_MESSAGE_INTERVAL` (511)**.  Using msg 30 and converting euler -> quaternion in the bridge avoids the request entirely | request capped at `loop_period / 0.8`; SITL's 400 Hz loop gives **up to ~320 Hz** |
| us -> FCU | **84** `SET_POSITION_TARGET_LOCAL_NED` | not rate-limited by the FCU.  **Position setpoints do NOT time out** -- `mode_guided.cpp:1194` is explicit that `GUID_TIMEOUT` is "only used for velocity, acceleration, angle control, and angular rates" | 50 Hz is comfortably fine |
| us -> FCU | **82** `SET_ATTITUDE_TARGET` | **this one IS angle/rate control**, so `GUID_TIMEOUT` (default 3 s) applies | 50 Hz fine |
| us -> FCU | **102** `VISION_POSITION_ESTIMATE` / **331** `ODOMETRY` | **no rate limit in AP_VisualOdom** (it keeps only a health timestamp) | 50 Hz fine; set `VISO_DELAY_MS` to our *measured* latency |

Consequences for the design:

* The FSM streams setpoints at **50 Hz**; the telemetry we depend on is raised to **`SR1_POSITION = 50`**
  and **`SR1_EXTRA1 = 50`** at start-up, and the bridge **verifies the achieved rates** (its own counter)
  rather than trusting the request.
* 50 Hz is the declared maximum of the stream-rate parameters, so 50 Hz is not a compromise -- it is the
  firmware's own ceiling for telemetry.
* Because **position** setpoints have no timeout, the "keep streaming while holding" rule is about
  smoothness and about not being in a velocity/attitude phase when the link hiccups -- not about keeping
  GUIDED alive.

## 7. TF tree

```
world_w  (W: world.yaml geometry, pillar markers)
   |  static, from Calib (yaw + translation, measured once per flight, checked against construction)
odom     (L: the LIO world -- also what the Path's frame_id, `lio_world`, means)
   |  from /odom at 200 Hz
base_link (B)
   |  static mount: 45 deg nose-down + (0.05, 0, 0.10)
mid360_link (S)
```

`map` is deliberately not used: the LIO has no loop closure or jumps, so one continuous frame suffices.
RViz shows the pillars (in `world_w`) and the cloud (in `odom`) together because of that static edge.  The
trajectory source and the FSM never touch W at flight time except through the measured `Calib`.

### Displaying it in RViz (asked: does anything need to be unified, will the cloud look tilted?)

Nothing needs to be *unified* -- that is what TF is for -- but one frame must be chosen and the tree must
reach it, and three things will look wrong if it does not:

1. **RViz Fixed Frame**: set it to **`world_w`** (`rviz/mission_a.rviz` does), so the world file's
   geometry, the pillars, the mission square and the cloud all appear in the frame the world is written in.
   (For debugging the raw sensor view, `mid360_link` is the other useful choice.)
2. **The raw scan is in S and S is pitched 45 deg.**  `/livox/lidar` is the housing frame, so displaying it
   without the `base_link -> mid360_link` static transform draws a cloud rotated 45 deg about the body's y
   axis -- the classic "the point cloud is tilted".
3. **Pillars are in W and the LIO's cloud is in L, and L is W rotated by a constant** (-90 deg with the
   North-facing spawn of `FRAMES.md` §2) **plus the ~0.3 m origin offset.**  Without the static
   `world_w -> odom` edge the two appear rotated by 90 deg relative to each other and offset -- exactly the
   artifact that looks like a broken cloud.
4. **One time base.**  TF lookups happen at the message's stamp, so if RViz and the publishers disagree
   about the clock (missing `use_sim_time`, missing `/clock`) the cloud flickers even though the frames are
   right.  `/clock` from box 2 is the single authority.

## 8. The transport interface (bidirectional, normalised)

Both implementations return the SAME numbers in the SAME frame: **`F`, ENU-labelled, metres, z up**,
quaternion = body(FLU) -> F.  The FCU's wire convention is reached inside the implementation, and each
implementation crosses exactly once.

```python
class Transport(Protocol):                       # both directions
    # outbound
    def arm(self, force=False) -> bool: ...
    def set_mode(self, mode: str) -> bool: ...
    def takeoff(self, alt_m: float) -> bool: ...
    def land(self) -> None: ...
    def send_position(self, p_f, yaw_rad=None) -> None: ...      # ENU labels, z up
    def send_position_velocity(self, p_f, v_f, yaw_rad=None, a_f=None) -> None: ...
    def send_velocity(self, v_f, yaw_rate=0.0) -> None: ...
    def send_external_nav(self, stamp, p_f, q_f, v_f) -> None:   # LIO -> FCU
    # inbound
    def state(self) -> VehicleState: ...          # connected / armed / mode / statustext
    def local_pose(self) -> Pose: ...             # msg 31+32, normalised to ENU / z up
    def local_velocity(self) -> Vec3: ...
```

| direction | `mavlink_impl` (pymavlink, the flown path) | a `mavros_impl` (NOT WRITTEN -- see above; this column is the contract it must meet) |
|---|---|---|
| setpoints out | `f_to_ned()` then msg 84 (z becomes DOWN) | publish `/mavros/setpoint_position/local` (mavros converts) |
| external nav out | ENU->NED + FLU->FRD + origin, then msg 102/331 | publish `/mavros/vision_pose/pose_cov` (mavros converts) |
| pose in | msg 32 + 31 -> `ned_to_f()` + `frd_to_flu()` | subscribe `/mavros/local_position/pose` (already ENU/FLU) |
| state in | `HEARTBEAT` + `STATUSTEXT` | `/mavros/state` + `/mavros/statustext/recv` |

**The trap this interface exists to kill:** `LOCAL_POSITION_NED.z` is DOWN, `/mavros/...pose.z` is UP.
"Hover at 2.5 m" is `-2.5` on one path and `+2.5` on the other, so an arrival check written against the
raw message is correct under one bridge and silently broken under the other.

## 9. Rates, latency, compute (measured where noted)

| stage | rate | cost | note |
|---|---|---|---|
| truth tap | **1200 Hz measured** (the model rate, `SIM_RATE_HZ`) | negligible | one UDP line per model step |
| lidar sim | 10 Hz x 20 000 rays | **9.8 ms/scan** (46 pillars) | 103 Hz capable; CPU is enough, GPU not needed |
| IMU sim | 200 Hz | negligible | generated from the tap |
| LIO | 10 Hz in, 200 Hz out | ~1 core | the dominant cost |
| rectangle source | publishes on start-up, then republishes | negligible | no planning, no map, no search |
| FSM | 50-100 Hz | negligible | arrival checks + setpoint streaming |
| FCU | 400 Hz loop | SITL built-in model | no Gazebo anywhere |
| end-to-end | -- | target < 100 ms | scan 100 ms + LIO + one setpoint period |

The two heavy consumers of the machine -- an ego-planner (1-2 cores, ESDF build) and its `planner_ws`
build -- are gone with the dropped line, which is also why `/sim/rtf` no longer has to be watched for
planner-induced collapse during a sortie (it still was, and it stayed at 1.000-1.002 in the last flights).

## 10. Deliberate differences from the old Gazebo pipeline

| old | here | why |
|---|---|---|
| Gazebo `gpu_lidar` at 500x40 rays, 2 cm range quantisation, physics-step IMU defect | our analytic ray caster, exact, real MID-360 pattern, our own noise model | the old sensor was the source of a phantom IMU step and of a starved matcher |
| `gz_livox_bridge` converting to `livox_ros_driver2/CustomMsg` | one `PointCloud2` in the velodyne layout, `lidar_type: 3` | the CustomMsg path in this fork **drops the per-point time**, i.e. no motion compensation |
| airframe bounding box removes self-hits | no airframe geometry at all -> no self-hits | the world is cylinders + planes; if the airframe is ever modelled, a body-exclusion step comes back |
| `/uav/*` normalisation layer, `/sensors/*` relay, orphan parameter files | one interface per function (the transport), no orphans | the old layer ended up referenced only by an orphan config |
| three terminals, MAVProxy router, `/uas1/mavlink_*` | one MAVLink endpoint to SITL plus the truth tap | fewer moving parts; the FSM owns the link |
| a planner in the loop | the square IS the trajectory; the FSM visits its corners | measured: the world's square holds **0.600 m** to the nearest pillar, i.e. the avoidance problem was already solved by the geometry; the planner's own value was never demonstrated (`STATUS.md`, last section) |

## 11. Safety gates (owned by `fsm/`)

1. **LIO liveness and rate**, before and during the flight: the FSM waits for the LIO in preflight
   (`lio rate >= 5 Hz` on a 0.9/0.1 smoothed estimate) and aborts with the number if it never arrives.
2. **Geofence**: the truth's |E| and |N| must stay inside `--geofence-m`, which
   `src/tools/world-info.py` derives from the world's own arena, so a gate cannot be tighter than the world the
   mission was built in.  Sim-only by construction, and labelled so: it reads the truth tap.
3. **Setpoint continuity**: stream at 50 Hz.  **Position** setpoints have no timeout at all, so this is
   about smoothness; the 3 s `GUID_TIMEOUT` applies to velocity/attitude control.
4. **Arrival is horizontal, with a documented vertical band.**  MEASURED 2026-10-13: a 3-D arrival test
   could never pass while the altitude loop ran on the barometer, and it read as an estimator bias when the
   horizontal error was 3-5 cm.  The corner is then held until the FCU's own horizontal speed is small,
   with a bounded wait.
5. **Bounded, attributed abort.**  Every abort names the mechanism and the numbers that produced it, and
   the mission's own timeouts (`--traj-wait-s`, the leg timeout, LAND's disarm wait) are all finite.  A
   wedge that leaves no line in the log is indistinguishable from a crash -- that lesson cost a 490 s
   silence once (`STATUS.md`, last section).
6. **No GPS, no compass**: arming depends on a healthy EKF (origin set, vision healthy).  A transiently
   invalid FCU state is tolerated for 3 s -- the old project lost 4 of 8 flights to one bad sample.

## EGO AS PUBLISHED versus THIS PROJECT'S ROUTE B -- two block diagrams, and where the defect lives

`docs/figures/ego-flow-comparison.png` (regenerable: `_stage/ego-flow.py`) draws both pipelines side by side,
stage by stage, with the measured number that decided each of our changes and, in red, the one remaining defect
attached to the stage that owns it.  The published pipeline is the one in *EGO-Planner: An ESDF-free
Gradient-based Local Planner for Quadrotors* (Zhou, Wang, Ye, Xu, Gao; arXiv:2008.08835) as it is implemented in
the `ego-planner-swarm` tree this project vendors.

**AS PUBLISHED.** `grid_map` is a LOCAL ROLLING BOX fed by a depth image, and it raycasts: cells the camera sees
through are CLEARED and cells it hits are OCCUPIED, so "free" means *seen and known clear*.  A distance-only A*
produces a global path inside that box; `getLocalTarget()` takes the point on it `planning_horizon` ahead;
`reboundReplan` initialises a B-spline and `BsplineOptimizer` -- ESDF-free, its collision term comparing the
trajectory against the A* guiding path -- minimises smooth + collision + feasibility + fitness + swarm; if the
result is dynamically infeasible `refineTrajAlgo` lengthens the time allocation and fits the higher derivatives
back on without changing the shape; `UniformBspline` is published and `traj_server` samples it at 100 Hz with
`t_cur = now - start_time`.  The FSM walks INIT -> WAIT_TARGET -> GEN_NEW_TRAJ -> REPLAN_TRAJ -> EXEC_TRAJ with a
1 s re-plan gate, declares arrival when the local target has caught up with the global one AND the clock is out,
and has EMERGENCY_STOP for "suddenly discovered obstacles".

THREE ASSUMPTIONS THAT DESIGN RESTS ON, all visible in the diagram, and none of which this project honours:
the map is dense and raycast (so nothing appears behind the vehicle), the controller follows the reference
tightly enough that the plan IS the flight, and a re-plan re-anchors the reference on the vehicle -- here
`t_cur` restarts at every commit, so the reference STEPS (measured: median 0.281 m, max 0.927 m, STATUS 155).

**WHAT WE RUN.**  `cloud_relay.py` replaces the depth image with a decimated, optionally unioned point cloud and
NO raycast, and the numbers are the whole of rounds 3-4: the planner's obstacle reached 0.720 m from a 0.300 m
pillar axis, which CLOSED the 1.414 m corridor between two diagonal pillars (-0.03 m) and made a
distance-optimal A* route around the entire field; voxel 0.03 and window 1.0 s brought it to 0.380 m and the
corridor to 0.654 m.  The map is rebuilt around the vehicle on every message, so it is a SINGLE-SCAN SHELL rather
than the dense map above.  ego gets no odometry of its own: `fcu_pose_feed.py` feeds it the flight controller's
own estimate (W -> L), so the planner and the controller close their loops on the same state.  `traj_server` is
ego's own, and `ego_contract_shim.py` owns the tour, the ids and the COMPLETED flag, and bounds the reference
with the position clamp, the slew limit, the tapered corner governor and the altitude band -- each of them
measured and switchable, in the same family as the "continuity at the source" fix the audits asked for.  The
bridge puts p + v + yaw on the wire at 50 Hz with the acceleration bit DROPPED, and the FCU's own shaper
(`shape_pos_vel_accel_xy`, WP_ACC 8.0 / PSC_NE_JERK 100 / ATC_ANGLE_MAX 40 deg) turns that into attitude.

**WHERE THE DEFECT IS, and it is not where the first six rounds looked.**  The map is measured free and
SYMMETRIC on all four commanded edges -- 0 occupied cells out of 1413 / 1679 / 915 / 914, while the pillar walls
0.5 m to either side carry 0.8-2.6 % (`_stage/edge-map.py`, STATUS 171).  An aircraft that follows the plan to
0.04-0.06 m on those legs still ends up **0.24-0.28 m OUTSIDE the y = +-4 edges, on the plan as well as on the
aircraft** (`_stage/edge-mean.py`), and raising `lambda_fitness` -- the only cost term that pulls the spline onto
the A* path -- moves those offsets without removing them, in both directions.  So the plan's lateral placement is
not determined by the mission anywhere in this chain: it is whatever the A* path and the spline settle on, and
it varies by +-0.3 to +-1.0 m between runs.  The geometry leaves 0.200 m.  **The budget is smaller than the
variation, which is why every lever tried in rounds 6-10 moved the outcome without making it deterministic** --
and why the remaining options are structural rather than tunable: bind the mission's line in the shell with a
corridor clamp, or give the route more room (there is none: 0.500 m from the pillar axes IS the widest
axis-aligned line a 1.0 m checkerboard allows).
