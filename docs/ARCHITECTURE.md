# ARCHITECTURE — boxes, links, topics, frames, interfaces, safety

One of the project's four living documents (budget in `PLAN.md` §7).  Everything here is a contract: rates,
frames and interfaces come from measurements or from the firmware source, and each conversion has a test
named in `FRAMES.md` §5.

**No Gazebo anywhere.**  The lidar is computed by our own analytic ray caster, the IMU is generated from
the simulator's true state, and RViz is used for display and for editing the world.  Where the old
Gazebo-based pipeline had `/mid360/*` (Gazebo Transport), `gz_livox_bridge`, `/sensors/*`, `/uav/*` and
orphan parameter files, this pipeline has exactly one interface per function and no orphans -- a parameter
file that nothing loads is a bug (verifiable by grep).

**No planner anywhere either, since 2026-10-15.**  An ego-planner was built behind the trajectory interface
of `PLAN.md` §4.1, flown, and then dropped by the owner; its files, its patches, its parameters, its RViz
view and the two tools that existed only to watch it left the tree in the same change
(`STATUS.md`'s last section keeps the measurement).  The boxes below are what remains, and the diagram is
the chain as it runs, not a design intent.

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
║ BOX 2 │  sim/           the world, the sensor, the clock, the truth receiver                 ║
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
║        /lio/cloud_world 10 Hz (L) = its VOXEL map (0.25 m), NOT a raw scan                ║
║        /lio/status 1 Hz, /tf (odom -> base_link)                                          ║
╚═══════════════╪═══════════════════════════════════════════════════════════════════════════╝
                │ pose + velocity (L)
                ▼
┌═══════════════════════════════════════════════════════════════════════════════════════════┐
║ BOX 4   planner/rectangle_source.py -- THE TRAJECTORY, and there is no other producer      ║
║   in : worlds/pillars.yaml through tools/world-info.py (the one geometry reader)           ║
║        --calib-file (_stage/calib.json): the SAME measured W<-L the FSM uses               ║
║   out: /planner/traj -- nav_msgs/Path in L, pose + yaw per point, per-point stamps         ║
║        one message per leg when RECT_HOLD_S > 0 (the corner hold), else one closed path    ║
╚═══════════════════════════════════════════════════════════════════════════════════════════╝
                │ /planner/traj (L)
                ▼
┌═══════════════════════════════════════════════════════════════════════════════════════════┐
║ BOX 5   fsm/mission_fsm.py + bridge/     scheduling, frames, arrival, FCU communication    ║
║   in : /planner/traj (L, latched)     <- the corners, converted L -> F once                ║
║        /lio/robo/odom 200 Hz (L)      /lio/odom (the velocity)                             ║
║        /sim/truth/odom (W)            <- the geofence gate ONLY (sim-only safety)          ║
║        FCU state/pose over MAVLink    [bridge]                                             ║
║   out: setpoint 84 @50 Hz             [bridge: F -> LOCAL_NED, z becomes DOWN]             ║
║        external nav 331 (+102) @48 Hz [bridge: LIO -> EKF, NED/FRD labels as AP requires]  ║
║        /fsm/phase (String) on change                                                       ║
║   owns: arrival at each corner, the settle, the truth-fed geofence, takeoff, LAND, and a    ║
║         BOUNDED abort that NAMES the mechanism instead of going silent                     ║
║   does NOT own: the trajectory (box 4), the estimate (box 3), the physics (box 1)           ║
╚═══════════════════════════════════════════════════════════════════════════════════════════╝

┌───────────────────────────────┐   ┌──────────────────────────────────────────────────────┐
│ BOX 6   RViz                  │   │ BOX 7   tools/                                       │
│  shows: /sim/world/markers(W) │   │  recorders: every stream + the truth -> logs/<run>/   │
│         /lio/cloud_world (L)  │   │  metrics.py: A1-A4 / B1-B4 / C1 / C2, offline         │
│         /planner/traj (L)     │   │  plot-run.py, dwell-audit.py, sortie-score.py         │
│         /tf, /odom, /fsm/phase│   │  frozen-check.py, docs-budget.py: the two rules,      │
│  frame: world_w (mission_a.rviz, FROZEN) │        executable                                    │
└───────────────────────────────┘   └──────────────────────────────────────────────────────┘
```

Frame shorthand: **W** world ENU (the file, origin = pad) · **L** LIO world (z up, origin = LIO init) ·
**F** FCU local (NED on the wire) · **B** body FLU · **S** MID-360 housing.  L and F are the same frame up
to axis labels; W differs from L by a known constant rotation + offset.  `FRAMES.md` owns the details.

## 2. Where the lidar and the IMU come from

**They are generated by us.**  With no Gazebo there is no sensor plugin, so `sim/` synthesises both from
two inputs: the **true vehicle state** (SITL's built-in quad model, read through the truth tap) and the
**world geometry** (`worlds/pillars.yaml`).

```
BOX 1 SITL: built-in quad model
        true state at each model step: p, v, q (body->NED), body rates w, specific force a
                 │
                 │ truth tap (sim-only, env-gated, 67 lines in ONE file, libraries/SITL/)
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
internally, and `sim/` generates the LIO's.  They derive from the same true motion, so both estimators see
consistent dynamics, but the noise realisations differ.  If that ever matters -- it would appear as an
attitude mismatch between `XKF1` and the LIO -- the fix is to make our IMU the single sensor and feed it to
SITL as well, which needs the external-sensor path (SIM_JSON) instead of the built-in model.

## 3. Per-box cards (what each box subscribes, publishes, owns)

| box | subscribes (rate, frame) | publishes (rate, frame) | owns |
|---|---|---|---|
| 1 SITL | MAVLink: setpoints 84, commands 76, external nav 102/331 | MAVLink: 31, 32, 0, 253; truth tap UDP **1200 Hz measured** | the true state, the EKF, the modes, the FCU local frame |
| 2 `sim/` | truth UDP 1200 Hz; `worlds/*.yaml` | `/livox/lidar` 10 Hz (S), `/livox/imu` 200 Hz (S), `/clock` 48 Hz, `/sim/rtf` 1 Hz, `/sim/truth/odom` 93 Hz (W), `/sim/world/markers` 1 Hz (W) | the world, the sensor models, the clock authority |
| 3 `lio/` | `/livox/lidar`, `/livox/imu` | `/lio/robo/odom` 200 Hz (pose only), `/lio/odom` 10 Hz (with velocity), `/lio/cloud_world` 10 Hz (the VOXEL map), `/lio/status`, `/tf` | the LIO world frame L and the odometry |
| 4 `planner/` | `worlds/*.yaml` (through `tools/world-info.py`), `--calib-file` | **`/planner/traj`** (`nav_msgs/Path`, L) | the trajectory — the square's four corners and, when asked, one message per leg for the corner hold |
| 5 `fsm/`+`bridge/` | `/planner/traj`, `/lio/robo/odom`, `/lio/odom`, `/sim/truth/odom` (geofence only), the FCU's pose/state | setpoints 84, external nav 331/102, `/fsm/phase` | scheduling, arrival, the safety gates, every conversion |
| 6 RViz | everything visual | nothing | display only (`rviz/mission_a.rviz` is frozen) |
| 7 `tools/` | everything (recorders) and every recording (offline) | logs, tables, figures | the evidence |

## 4. The chain AS BUILT — measured, 2026-10-15

Every row is a measured number from a run directory, and the row says where.

| stage | what really happens | the number, and where it came from |
|---|---|---|
| truth | ArduPilot SITL (Copter-4.7.1 + `patches/apm-0001-sitl-truth-tap.patch`) steps at 1200 Hz and emits `TAP` lines on UDP | **1200.5 Hz**, `logs/s3-parked-*`; against the firmware's own SIM2 record: position max **0.0004 mm**, clock bias +0.0000 ms, drift +0.06 ppm |
| sensor | `sim/lidar_imu.py` renders `/livox/lidar` (20 000 points, 10 Hz, per-point times) and `/livox/imu` | **199.4-200.1 Hz IMU**, the decimation derived from the rate that ARRIVES, `logs/missionA-imu200a` |
| LIO | clean upstream Super-LIO `f89f48d`: `/lio/robo/odom` (200 Hz, body pose, L, zero twist), `/lio/odom` (10 Hz, with velocity), `/lio/cloud_world` (the 0.25 m voxel map) | 200.1 Hz delivered, `logs/missionA-stepB10-*` |
| FCU feed | the FSM re-samples the newest LIO pose in its own 50 Hz loop and streams ODOMETRY (331) in NED/FRD | the FCU's own log: **VISP/VISV at 48.2 Hz**, `tools/bin-extnav-rate.py` |
| EKF | `EK3_SRC1_POSXY/POSZ/VELXY/VELZ/YAW = 6` (external nav), no GPS, no compass; every measurement and process noise set to the MEASURED value in `params/ekf_trust_lio.parm`, with `FS_EKF_THRESH` raised in proportion (it is a RATIO) | `tools/bin-params.py` on the flight BIN |
| trajectory | `planner/rectangle_source.py` converts the world file's square W -> L with the run's measured calibration and publishes it on `/planner/traj`; `RECT_HOLD_S=5` publishes one leg per message and keeps publishing it past its flight time | W->L->W round trip printed in `traj.log`, worst **0.000000 mm**; the published plan's own clearance, read back in W: **0.6000 m** (the world's designed worst case) |
| mission | the FSM visits the path's **corners** (`--follow corner`, the default and what every table in `docs/metrics/` was measured with): velocity-free position setpoints, a horizontal arrival test with a documented vertical band, and a settle before turning in | A1 4.65-4.90 mm, A4 18.6-21.1 mm, **10 of 10 flights**, `docs/metrics/stepB-rectangle-20261013.txt` |
| last flight with no ego node (RECT1-noego) | the whole pipeline with zero planner processes | A1 4.40 mm, A2 5.96 mm, A4 20.72 mm, **C2 min 0.547 m with 0 of 28425 samples inside the 0.20 m footprint**, C1 0.022 m, B4 landed |
| scoring | `tools/metrics.py`: A1/A2 (LIO vs truth), A3 (delivered-stream continuity, bounds anchored to the acceptance table), A4 (the FCU's own `XKF1` vs truth, clock-aligned, best-fit lag reported), B1-B4 (truth vs the commanded geometry), **C1** final point, **C2** minimum truth-to-pillar-surface distance from the world file | every line prints its denominator; `tools/batch-summary.py` aggregates them; `tools/dwell-audit.py` measures the corner dwell from the tap |

**THREE RULES THIS STACK PAID FOR**, each a class rather than an incident:

1. **One geometry source.**  `tools/world-info.py` is the only reader of the world file; the sensor, the
   trajectory source, the FSM's geofence and the metrics all take their geometry from it.  MEASURED:
   `run-mission.sh` once started the sensor WITHOUT `--world`, so it rendered `worlds/pillars.yaml` while
   everything else used another forest -- two worlds, and the plans were judged against pillars the
   aircraft never flew near.  The same rule now also covers the pillar CLEARANCE itself: one definition,
   `tools/world-info.py::PillarWorld`, imported by everything that prints a distance.
2. **An interface has three properties: name, type and QoS** -- and a fourth one here: **one producer.**
   A latched, RELIABLE Path is the whole contract; the FSM does not know whether it comes from a file, a
   mission script or a planner, which is why dropping the planner cost the FSM nothing.
3. **A number carries its rate, its frame, its window -- and its DELAY is a number too.**  A3's bound was
   written "per sample at 200 Hz" for a stream arriving at 33.3 Hz; A4 grows with speed because the
   velocity is ~80 ms old, and its gate now scores the residual at the best-fit lag rather than at zero.

**AND ONE MORE, ADDED 2026-10-15: a producer's own log is not evidence of what the aircraft did.**  The
corner-hold producer logged three 5 s holds while the truth track spent them somewhere else entirely
(`tools/dwell-audit.py`, measured on `tap.csv`).  Every claim about the flight is measured from the truth
tap; the producer's log says only what the producer intended.

## 5. Topics — rate, content, frame

| topic | type | rate | content | frame | producer -> consumer |
|---|---|---|---|---|---|
| truth tap (UDP, not ROS) | ASCII line | **1200 Hz measured** (the model rate) | `TAP <inst> <t_us> p_ned v_ned q(w,x,y,z body->NED) gyro FRD accel FRD` | SITL native, declared | box 1 -> box 2 |
| tap fan-out (UDP, local) | the same lines | 1200 Hz, decimated to 200 for the sensor | so `sim/lidar_imu.py` can consume it as a second process (only one process may bind the tap's port) | as above | box 2 -> box 2 |
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
| `/planner/traj` | **`nav_msgs/Path`** | per-point stamps; republished while the source runs; latched (TRANSIENT_LOCAL, depth 20 on the consumer) | pose + yaw per point, each point stamped with the instant the mission means it to be reached.  Dense enough to difference for velocity | **L** | box 4 -> box 5 |
| `/fsm/phase` | `std_msgs/String` | on change + 1 Hz | preflight / takeoff / hold / leg1..4 / land / abort / done | -- | box 5 -> box 7 (window segmentation) |
| setpoint 84 | MAVLink | **50 Hz** | sampled position (+ optional velocity/accel/yaw/yaw_rate), `LOCAL_NED` | F, NED on the wire | box 5 -> box 1 |
| command 76 | MAVLink | on demand | arm / takeoff / land / mode / simple accel calibration | -- | box 5 -> box 1 |
| vision 102 / odometry 331 | MAVLink | **48.2 Hz measured at the FCU** (`VISP`/`VISV` in its own log) | LIO pose + velocity (+attitude) | F, NED on the wire with the FRD *labels* ArduPilot requires | box 5 -> box 1 |
| telemetry 32 / 31 | MAVLink | **50 Hz** | FCU local position/velocity (**z DOWN**) / attitude (FRD body -> NED) | F / B | box 1 -> box 5 |
| 0 / 253 | MAVLink | 1-2 Hz / on event | armed, mode, health / text | -- | box 1 -> box 5 |
| `/planner/stalled` | `std_msgs/Bool` | -- | **KEPT, NEEDS A DECISION**: a trajectory source saying "I have stopped producing plans".  The FSM still subscribes and still turns it into a named abort, but the dropped planner chain was its only publisher, so today nothing raises it | -- | (nobody) -> box 5 |

With `BRIDGE=mavros` the MAVLink rows become mavros topics (`/mavros/local_position/pose` 30-50 Hz ENU,
`/mavros/state` 1 Hz, setpoints on `/mavros/setpoint_position/local`, vision on
`/mavros/vision_pose/pose_cov`) and **mavros performs the frame conversions**.  `BRIDGE=mavlink`
(pymavlink) is the flown path; `mavros_impl` exists and is unflown.

### Two disciplines the old pipeline taught, kept here

* **Rates are judged by the publisher's own counters, not by `ros2 topic hz`.**  Measured in the old
  project: two consecutive `topic hz` readings of the same topic differed by more than 3x under load.
  Every rate-sensitive node reports its achieved rate in its own status topic, and the metrics read those.
* **No orphan parameter files.**  In the old tree a parameter file existed that no launch path loaded, and
  reasoning from it gave wrong conclusions about which topics were live.  Every file under `params/`,
  `sim/`, `lio/`, `planner/` must be referenced by a launch path, checked by grep in review.

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

| direction | `mavlink_impl` (pymavlink, the flown path) | `mavros_impl` (written, unflown) |
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
   `tools/world-info.py` derives from the world's own arena, so a gate cannot be tighter than the world the
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
