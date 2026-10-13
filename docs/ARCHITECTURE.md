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
║          └──────────────► TRUTH TAP (sim-only, env-gated) ──┼──► UDP 400 Hz              ║
╚═════════════════════════════════════════════════════════════╪═════════════════════════════╝
        ▲ MAVLink 5760/udp:  setpoints 84 @20-50 Hz, commands 76        │ truth: p/v NED,
        │                    telemetry 31/32 @20-50 Hz, 0/253 @1-2 Hz   │ q FRD, gyro, accel
        │                                                               ▼
┌═══════╪═════════════════════════════════════════════════════════════════════════════════════┐
║ BOX 2 │  sim/           the world, the sensor, the clock, the world editor                 ║
║   ┌───┴──────────┐   ┌──────────────────┐   ┌─────────────────┐   ┌────────────────────┐  ║
║   │ truth_rx     │──►│ lidar_imu        │   │ clock           │   │ world editor       │  ║
║   │ (UDP -> ROS) │   │ raycast + noise  │   │ /clock 100 Hz   │   │ RViz clicks ->     │  ║
║   │ 400 Hz       │   │ IMU model        │   │ /sim/rtf 1 Hz   │   │ worlds/*.yaml      │  ║
║   └──────────────┘   └────────┬─────────┘   └─────────────────┘   └─────────▲──────────┘  ║
║   worlds/pillars.yaml ────────┘  (geometry, mount 45 deg, mission, fake_truth)│           ║
╚════════════════════════════════╪══════════════════════════════════════════════╪═══════════╝
     /livox/lidar 10 Hz (S)      │      /livox/imu 200 Hz (S)                   │ /clicked_point (W)
     /sim/truth/odom 100 Hz (W)  │                                              │
                ┌────────────────┴────────────────┐                             │
                ▼                                 ▼                             │
┌═══════════════════════════════════════════════════════════════════════════════╪═══════════┐
║ BOX 3   lio/          one LIO process (Super-LIO or FAST-LIO) + adaptation    │           ║
║   in : /livox/lidar 10 Hz (S), /livox/imu 200 Hz (S)                          │           ║
║   out: /lio/odom 10 Hz (L, raw state)                                         │           ║
║        /odom 200 Hz (L, fitted, child = base_link)                            │           ║
║        /cloud_registered 10 Hz (L, scan in the map frame)                     │           ║
║        /lio/map 0.2-1 Hz (L, optional), /lio/status 1 Hz (rate, degeneracy)   │           ║
║        /tf  (odom -> base_link)                                               │           ║
╚═══════════════╪═══════════════════════════════════════════════════════════════╪═══════════╝
                │ /odom 200 Hz (L)          /cloud_registered 10 Hz (L)         │
                ├──────────────────────────────┬───────────────────────────────┘
                │                              ▼
                │            ┌═════════════════════════════════════════════════════════════┐
                │            ║ BOX 4   planner/   ego-planner                              ║
                │            ║   in : /odom 200 Hz (L)   <- state, same source as the map  ║
                │            ║        /cloud_registered 10 Hz (L)  <- local map input      ║
                │            ║        /planner/goal (L, on demand, W->L by the FSM)        ║
                │            ║   out: /planner/traj 10-50 Hz (L)  B-spline horizon        ║
                │            ║        /planner/status 1-10 Hz, /planner/vis/* 10 Hz        ║
                │            ╚═══════════════════════════════════╪═════════════════════════╝
                │                                             │ /planner/traj (L)
                ▼                                             ▼
┌══════════════════════════════════════════════════════════════════════════════════════════┐
║ BOX 5   fsm/  +  bridge/        scheduling, frames, arrival, FCU communication            ║
║   in : local_pose (msg 31/32 -> F, ENU labels, z up) @20-50 Hz      [bridge]              ║
║        state (HEARTBEAT/STATUSTEXT or /mavros/state) @1-2 Hz        [bridge]              ║
║        /odom 200 Hz (L)            <- for logging and the sanity gate                     ║
║        /planner/traj 10-50 Hz (L)  <- sampled into setpoints                              ║
║        /lio/status 1 Hz            <- gate: rate, jump, degeneracy                        ║
║        worlds/*.yaml mission (W)   <- the mission it schedules                            ║
║   out: setpoints (setpoint 84) @20-50 Hz   [bridge: normalised F -> NED on the wire]      ║
║        external nav (msg 102/331) @20-100 Hz  [bridge: LIO -> FCU]                        ║
║        /planner/goal (L) on demand, /fsm/phase (String) on change + 1 Hz                  ║
║        tf: world_w -> odom  (static, from Calib, measured once per flight)                ║
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
                 │ UDP 400 Hz, ASCII, frame declared in the header: NED / FRD
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
| 1 SITL | MAVLink: setpoints 84, commands 76, external nav 102/331 | MAVLink: 31, 32, 0, 253; truth tap UDP 400 Hz | the true state, the EKF, the modes, the FCU local frame |
| 2 `sim/` | truth UDP 400 Hz; `/clicked_point` (W); `worlds/*.yaml` | `/livox/lidar` 10 Hz (S), `/livox/imu` 200 Hz (S), `/clock` 100 Hz, `/sim/rtf` 1 Hz, `/sim/truth/odom` 100 Hz (W), `/sim/world/markers` 1 Hz (W) | the world, the sensor models, the clock authority |
| 3 `lio/` | `/livox/lidar`, `/livox/imu` | `/lio/odom` 10 Hz, `/odom` 200 Hz, `/cloud_registered` 10 Hz (all L), `/lio/map`, `/lio/status`, `/tf` | the LIO world frame L and its odometry |
| 4 `planner/` | `/odom`, `/cloud_registered`, `/planner/goal` | `/planner/traj`, `/planner/status`, `/planner/vis/*` | the trajectory and its own ESDF/grid |
| 5 `fsm/`+`bridge/` | FCU pose/state, `/odom`, `/planner/traj`, `/lio/status`, the mission file | setpoints, external nav, `/planner/goal`, `/fsm/phase` | scheduling, arrival, the gates, every conversion |
| 6 RViz | everything visual | tool topics | nothing (display and world editing only) |
| 7 `tools/` | everything | logs, tables, figures | the evidence |

## 4. Topics — rate, content, frame

| topic | type | rate | content | frame | producer -> consumer |
|---|---|---|---|---|---|
| truth tap (UDP, not ROS) | ASCII line | **400 Hz** | t, p_ned, v_ned, q(w,x,y,z body->NED), gyro FRD, accel FRD | SITL native, declared | box 1 -> box 2 |
| `/clock` | `rosgraph_msgs/Clock` | 100 Hz | sim time (the single time authority) | -- | box 2 -> all |
| `/sim/rtf` | `std_msgs/Float64` | 1 Hz | sim seconds per wall second, plus the achieved lidar/IMU rates | -- | box 2 -> box 7 |
| `/livox/lidar` | `sensor_msgs/PointCloud2` | **10 Hz** | 20 000 points; x,y,z,intensity,time; `time` = per-point offset 0..0.1 s | **S** | box 2 -> box 3 |
| `/livox/imu` | `sensor_msgs/Imu` | **200 Hz** | gyro rad/s, accel m/s^2, no orientation | **S** | box 2 -> box 3 |
| `/sim/truth/odom` | `nav_msgs/Odometry` | 100 Hz | true body pose/velocity (never used for control) | **W** | box 2 -> box 7 |
| `/sim/world/markers` | `visualization_msgs/MarkerArray` | 1 Hz / on change | pillars, arena, the clear rectangle | **W** | box 2 -> RViz |
| `/clicked_point` | `geometry_msgs/PointStamped` | on click | operator adds or moves a pillar | **W** | RViz -> box 2 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | on click | operator sets the mission-B goal | **W** | RViz -> box 5 |
| `/lio/odom` | `nav_msgs/Odometry` | **10 Hz** | ESKF state at scan rate, child `mid360_link` | **L** | box 3 -> box 7 |
| `/odom` | `nav_msgs/Odometry` | **200 Hz** | fitted body pose + velocity, child `base_link` | **L** | box 3 -> box 4, box 5, RViz |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | **10 Hz** | the scan registered into the map | **L** | box 3 -> box 4, RViz |
| `/lio/map` | `sensor_msgs/PointCloud2` | 0.2-1 Hz | accumulated map (optional) | **L** | box 3 -> RViz |
| `/lio/status` | `diagnostic_msgs/DiagnosticArray` | 1 Hz | measured scan rate, effective points, degeneracy lambda2/lambda1, map size | -- | box 3 -> box 5 gate, box 7 |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | 200 Hz / static | `world_w -> odom -> base_link -> mid360_link` | -- | box 5 (static pair), box 3 (odom edge) |
| `/planner/goal` | `geometry_msgs/PoseStamped` | on demand | next goal, converted W -> L by box 5 | **L** | box 5 -> box 4 |
| `/planner/traj` | `traj_utils/Bspline` | **10-50 Hz** | horizon: control points, start/end time, yaw | **L** | box 4 -> box 5 |
| `/planner/vis/*` | `visualization_msgs/MarkerArray` | 10 Hz | goal, path, ESDF slice | **L** | box 4 -> RViz |
| `/planner/status` | `std_msgs/String` | 1-10 Hz | replan ok/fail + reason | -- | box 4 -> box 5 |
| `/fsm/phase` | `std_msgs/String` | on change + 1 Hz | preflight / takeoff / hold / leg1..4 / land / abort | -- | box 5 -> box 7 (window segmentation) |
| setpoint 84 | MAVLink | **20-50 Hz** | sampled position (+ optional velocity/accel/yaw/yaw_rate), `LOCAL_NED` | F, NED on the wire | box 5 -> box 1 |
| command 76 | MAVLink | on demand | arm / takeoff / land / mode | -- | box 5 -> box 1 |
| vision 102 / odometry 331 | MAVLink | **20-100 Hz** | LIO pose + velocity (+attitude) | F, NED on the wire | box 5 -> box 1 |
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
and the planner never touch W except when the mission is set up.

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
| truth tap | 400 Hz | negligible | one UDP line per model step |
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
3. **Setpoint continuity**: stream at >= 20 Hz even while holding (`GUID_TIMEOUT` is 3 s).
4. **Planner failure**: if `/planner/status` reports failure for > 1 s while moving, hold; if it
   persists, fly straight to the goal slowly.
5. **No GPS, no compass**: arming depends on a healthy EKF (origin set, vision healthy).  A transiently
   invalid FCU state is tolerated for 3 s -- the old project lost 4 of 8 flights to one bad sample.
