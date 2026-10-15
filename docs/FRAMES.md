# FRAMES — the coordinate conventions of this stack

The decisions taken from this discussion are recorded in `PLAN.md` §2; this file owns the conventions and
the tests that pin them.  `ARCHITECTURE.md` says how the pieces fit, `STATUS.md` says where we are.

**The chain this file governs, in one line:**

```
W  --[common/frames.py + the run's Calib]-->  L  --[the FSM's L->F]-->  F  --[the bridge]-->  NED/FRD on the wire
world ENU, origin = pad          LIO world, z up          FCU local (NED labels)      what ArduPilot reads
```

Four conversions, and each has one owner.  The W <-> L pair is used by four files since 2026-10-15 -- the
corner hold made the producer an L -> W reader of its own odometry, and the trajectory recorder was computing
a W-frame clearance on an L-frame path -- and every one of them takes the SAME `--calib-file`: one
convention, one constant.

| # | conversion | where it happens | pinned by |
|---|---|---|---|
| 1 | **W -> L** (the mission's geometry) and **L -> W** (what comes back) | W->L: `planner/rectangle_source.py` at publish time, with `--calib-file`; the same constant in `tools/world-info.py`/`tools/metrics.py` when a recording is read back.  L->W: the same file for the corner hold's arrival gate (`--odom`, 2026-10-15), `tools/record-traj.py` before it computes `clr_m`, and `tools/plan-vs-truth.py` / `tools/plot-run.py` when a recording's per-point `frame` column says L | the round trip printed in `traj.log` (worst 0.000000 mm); `tools/traj-clearance-audit.py` (a fresh recording: recorded column == independent recomputation to 0.000000 m; RECT6's old one: -0.2972 -> +0.6000 m); `docs/metrics/stepB-rectangle-20261013.txt` |
| 2 | **L -> F** (the LIO's pose, and the path the FSM flies) | `fsm/mission_fsm.py`: `stream_external_nav()` for the FCU feed, `legs()` for the path.  The steady-state loop runs entirely in F | A1/A2 (LIO vs truth), A4 (FCU vs truth), and the measured `Calib.t_WL` |
| 3 | **F -> NED / FLU -> FRD** (the wire) | `bridge/mavlink_impl.py` via `common/frames.py`; with `BRIDGE=mavros`, **mavros does it** and the implementation must NOT also flip | the S7 link test and, in flight, the FCU's own `XKF1` vs the truth (A4) |
| 4 | **B <-> S** (the 45 deg mount) | `common/frames.py::R_body_sensor()`, applied by `sim/lidar_imu.py` to the ray directions and declared to the LIO as `lio.extrinsic.odom_robo` | the S4 surface split and the pillar fit (`tools/sim-pillar-gate.sh`), and T3's 45.000 deg |

## 0. HOW MANY FRAMES (the honest answer)

**Six frames EXIST.  Three of them are the ones our own logic has to hold at the same time.  Four
conversions must be written; three are fixed constants, one is the FCU's wire contract.**

NED and FRD are **wire contracts**, and the code makes them two *different* contracts.  See §1.

### 0.1 The six frames that exist

| # | frame | where it appears | whose definition | can it be collapsed? |
|---|---|---|---|---|
| 1 | **FCU local NED** | `SET_POSITION_TARGET_LOCAL_NED` out, `LOCAL_POSITION_NED` / `XKF1` in | ArduPilot, origin = EKF origin | no -- it is the control interface |
| 2 | **FCU body FRD** | `ATTITUDE` telemetry, `ODOMETRY`'s `child_frame_id` | ArduPilot | no -- attitude interface |
| 3 | **our world ENU** | `worlds/pillars.yaml`, ray casting, mission geometry | us | it IS the reference |
| 4 | **body FLU** | ROS `base_link`, RViz, our sim | ROS/REP-103 | no -- the body frame the odometry is published in |
| 5 | **MID-360 housing** | the pattern CSV, the LIO's lidar+IMU input | Livox | = frame 4 + fixed mount (keep as a constant) |
| 6 | **LIO world** | the LIO's odometry output, and the `frame_id` of `/planner/traj` (`lio_world`) | the LIO's initialisation | NOT merged into frame 3: it differs by a measured constant, see §2 |

"Three" is the count of frames **our logic must reason about simultaneously**: the world (3, with 6
reachable through one measured constant), the body (4, with 2 as its FRD alias), and the sensor (5, a fixed
offset of 4).

### 0.2 What the source says about frames on the wire (Copter-4.7.1, read 2026-10-13)

* **Setpoints** (`ArduCopter/GCS_MAVLink_Copter.cpp:1010-1013`): Copter accepts exactly four frames --
  `MAV_FRAME_LOCAL_NED`, `MAV_FRAME_LOCAL_OFFSET_NED`, `MAV_FRAME_BODY_NED`, `MAV_FRAME_BODY_OFFSET_NED`.
  Everything else is refused.
* **`ODOMETRY` input** (`GCS_MAVLink/GCS_Common.cpp:4062-4065`):
  `if (m.frame_id != MAV_FRAME_LOCAL_FRD || m.child_frame_id != MAV_FRAME_BODY_FRD) return;` -- with the
  comment "only support local FRD frame data".  **A wrong frame id does not degrade the estimate: the
  message is dropped entirely.**
* **... and yet the numbers are consumed as NED.**  `handle_odometry` passes `m.x/m.y/m.z` straight to
  `AP_VisualOdom_MAV::handle_pose_estimate`, which applies only a scale factor and a body-offset correction
  and then calls `AP::ahrs().writeExtNavData(pos, attitude, ...)`, whose position input is NED.  So the
  enum name `LOCAL_FRD` is a *label requirement*, not the frame of the numbers we send: **send NED numbers,
  label them `LOCAL_FRD`/`BODY_FRD`.**  Its **velocity** is different again -- `m.vx/vy/vz` is body-FRD and
  ArduPilot rotates it by the quaternion before use.
* `VISION_POSITION_ESTIMATE` carries no frame field at all; its x/y/z are taken as NED position and its
  roll/pitch/yaw as the vehicle attitude.

**Consequence: "send NED" is right for setpoints and for the vision position, but the `ODOMETRY` message
additionally demands FRD frame *labels*, and its velocity channel is body-FRD.**  One message, three
conventions.  This is the single most likely place for a silent sign/frame error, and it is why the move
tests in §5 are not optional.  What is measured in flight: **A4 = the FCU's own estimate vs the truth**,
18.6-21.1 mm horizontal over 10 flights -- if a sign or a label were wrong, that number would not be small.

### 0.3 Making L and W coincide on purpose — and why they do not (the user's question)

The fork's `kf_init` aligns gravity and then *removes the vehicle's heading* (`yaw = atan2(n.y, n.x)` of
the body x-axis, then `rot = g_lidar_robo_yaw * R_yaw_inv * init_rot`), so **L's x-axis is the body's
forward axis at the instant the LIO initialises**, and L's origin is the housing's position then.

**The vehicle spawns facing North (decision 14)**, because that makes the FCU link the textbook
permutation `(x_ned, y_ned, z_ned) = (N, E, -U)` with no yaw parameter anywhere in the mission path -- and
the FCU link is where the mission is sent and where a silent rotation flies the square rotated while every
internal check passes.  With that choice, **L and W do NOT coincide**: they differ by a constant -90 deg
yaw (L->W is +90) plus a translation of the LIO's init position.

Direction convention, because this is exactly where a sign hides: `R_WL` maps **L coordinates to W**
(`v_W = R_WL @ v_L`), so `R_WL[:,0]` is L's x-axis written in W.  Facing North that column is `(0, 1, 0)`
and `yaw(R_WL) = +90 deg`; the inverse map `R_LW = R_WL.T` has yaw -90 deg.  Both numbers appear in this
project's notes, so `frames.py` says which one every function returns, and `Calib.yaw_deg()` reports
`R_WL`'s yaw.

**Note added 2026-10-13 (the module's own test caught it):** "North" here is a **compass** heading
(0 = North), while a rotation about W's z is an **ENU** yaw (0 = East, anticlockwise).  They differ by
90 deg and the same word "yaw" is used for both in older notes.  `frames.py` therefore has
`R_WB_from_heading(heading_deg)` for the compass reading and `R_z(angle_rad)` for the ENU one, and never
takes one where the other belongs.

**What is left, and what each part costs:**

1. **Origin.**  L's origin is the LIO's init position, which the fork sets to the lever arm
   `g_odom_robo.t_ = (0.05, 0, 0.10)` -- about 0.3 m above the pad.  It is subtracted in one place (the
   FSM's `stream_external_nav`, through `Calib.t_WL`), and the constant is what T5 exists to pin.
2. **The spawn condition is declared and checked, not assumed.**  `worlds/pillars.yaml` carries an explicit
   `spawn:` block (position + yaw), and the FSM asserts from the truth tap that the vehicle really was
   parked, level and at that heading when the LIO initialised.
3. **What the constant does NOT fix.**  It is a frame statement, not an estimator statement: the 2.5 deg
   pitch of the OLD LIO's estimate (which A2's 31-37 mm was made of) is a property of that estimator, and
   the current upstream LIO does not show it -- A2 on the LIO's own altitude is 4.56-4.89 mm over 10
   flights.

### 0.4 Where B (body FLU) is used, and why it cannot be merged — but can be demoted

**B is used in exactly three places:**

1. **The parent of the sensor mount**: `S = B` rotated by the 45 deg nose-down mount and offset by
   (0.05, 0, 0.10).  Without B there is nothing to hang the housing on.  Sign, stated once: a nose-down
   mount means the housing's forward axis lies **below** the body's, i.e. in body coordinates
   `x_S = (cos 45, 0, -sin 45)` and `R_BS = R_y(+45)` -- the housing's *up* axis tilts forward.  The LIO
   stack states the same mount as `odom_robo` pitch +45 with its rpy applied inverted, so the two
   documents disagree in words and agree in physics; `R_body_sensor()` encodes the physics and T3 measures
   it (45.000 deg) instead of trusting either wording.
2. **The published odometry frame**: `/odom` (after the LIO applies `odom_robo`) is the body pose -- what
   RViz draws.  `/lio/odom` is the same state in S.
3. **The FCU's external-nav attitude**: ArduPilot wants the *body* attitude; its own body convention is
   FRD, which is B under a fixed 180 deg roll.  (`VISO_POS_X/Y/Z = 0`, i.e. we send the body origin, not
   the sensor's.)

**It cannot be merged into W or L**, and the reason is not bookkeeping: W and L are *fixed* frames, B is
the *moving* one.  Its pose in L is exactly what the LIO measures; merging would assert the vehicle never
rotates.  **But it can be demoted:**

| frame | who ever mentions it | why |
|---|---|---|
| **W** | the mission file, the trajectory source at publish time, RViz | the world geometry lives here |
| **L** | LIO output, `/planner/traj`, the FSM loop, the metrics | the map and the state live here |
| **B** | `/odom`, `base_link`, the external-nav message | one rigid offset from S, one 180 deg roll from the FCU's body |
| **S** | only `sim/lidar_imu` (publishes here) and the LIO (consumes here) | an implementation detail of the sensor + LIO pair |

So the chain reads: **sim generates in S -> the LIO consumes S and publishes B (in L) -> the mission speaks
W once, at publish time, and F everywhere else.**  Only two files in the project ever name S, and one
number (the mount) connects S to B.

**The failure this demotion must not hide**: feeding the FCU the *sensor* attitude instead of the *body*
attitude.  The vehicle would look pitched 45 deg to its own controller, which would then try to correct
it.  The parked attitude test (T3) is where that is caught, in its sharp form: **on the pad the body's
pitch must be ~0, never ~45 deg**, and the two are measured side by side (`body pitch 0.000 deg` while
`specific force in S is 45.000 deg from the housing's +z`).

## 1. The frames that exist

| # | frame | axes | origin | produced by | consumed by |
|---|---|---|---|---|---|
| F1 | SITL / ArduPilot local **NED** | x North, y East, z Down | EKF origin — **must be set explicitly** (no GPS here) | ArduPilot | GUIDED setpoints, the A4 comparison |
| F2 | ArduPilot **body FRD** | x fwd, y right, z down | vehicle CG | ArduPilot | logged attitude, the external-nav labels |
| F3 | **world.yaml / sensor-sim world (W)** | x East, y North, z Up | the pad | our sim | ray casting, mission geometry, RViz |
| F4 | our sim **body FLU (B)** | x fwd, y left, z up | body origin | our sim | the mount transform, `/odom` |
| F5 | **MID-360 housing (S)** | x fwd, y left, z up, mounted **45 deg nose-down** | housing centre | our sim (cloud + IMU) | the LIO |
| F6 | **LIO world (L)** | gravity-aligned, z up, **yaw = the body's heading at init** | where the LIO initialised | Super-LIO | the FSM, the metrics, `/planner/traj` |
| F7 | the FCU's local frame **as the EKF sees it** | NED, **yaw-locked to F6** (`EK3_SRC1_YAW = 6`) | EKF origin | ArduPilot | GUIDED setpoints |
| F8 | ROS / RViz | ENU + FLU (REP-103/105) | `world_w` / `odom` | us | RViz |

## 2. What is already MEASURED (do not re-litigate)

* **F5's azimuth convention.** `sim/raycast.py` uses `point.x = radial_xy * cos(h)`,
  `point.y = radial_xy * sin(h)` -- azimuth measured from +x toward +y, counter-clockwise seen from above --
  and the real `scan_mode/mid360.csv` agrees with it (MEASURED here: reading that column as elevation puts
  the fan in the wrong half of the sky; the file's own numbers are elevation -7.16..+52.16 deg after
  conversion).
* **F5 carries BOTH sensors.** `livox_ros_driver2` sets one `frame_id` for the cloud and the IMU and copies
  gyro/accel with **no rotation**, so the lidar and the IMU are one rigid housing:
  `lio.extrinsic.lidar_imu` is **identity**.
* **The mount is NOT in that extrinsic.** The 45 deg nose-down mount belongs to lidar->body
  (`lio.extrinsic.odom_robo`), whose documented sign convention is that the transform actually applied is
  the INVERSE of the stated rpy: to get `Ry(-45)` applied, pass pitch `+45`.  Encoding the mount in
  `lidar_imu` "would inject a spurious 45 deg on the real drone".
* **F6's yaw is set by the spawn, and the spawn is asserted.**  MEASURED across five flights of the OLD
  stack, the rotation between the LIO's world and the simulator's was -89.7, +115.7, -88.0, -11.3,
  -87.9 deg -- i.e. arbitrary, because the LIO's world is gravity-aligned only.  Here the vehicle is
  spawned facing North and the condition is **checked from the truth tap before take-off**, so `Calib` has
  one known constant to verify instead of an unknown rotation to fit.  The measured value in flight:
  `R_WL` yaw **+90.000 deg** (predicted) vs **+90.050 deg** (fitted from a calibration flight).
* **Therefore F7 is rotated relative to F3 by that same constant** -- not an unknown per-run angle.  With
  `EK3_SRC1_YAW = 6` the EKF *adopts* F6's yaw, so the FCU's local NED **is** F6 with different labels, and
  no frame alignment inside the loop is needed anywhere: only the labels differ (`R_LF` is the identity up
  to the ENU/NED label permutation, `frames.enu_ned()`).
* **The 45 deg mount is SETTLED (user decision, 2026-10-13).** MEASURED before deciding: at 0.5 m sensor
  height the 45 deg nose-down fan returns **29.6 % floor / 16.8 % ceiling / 44.9 % pillars**; a level mount
  gives **4.7 % / 7.1 % / 74.3 %** -- i.e. essentially no floor, and the vertical channel would lose its
  best constraint, which matters because the mission's height comes from the LIO.

## 3. The traps

* **T1 — mission geometry lives in the wrong frame if you are not careful.** We define the square in W.
  The GUIDED setpoints we send are in F7 = F6 = the LIO's frame.  Without the measured constant the square
  comes out rotated by ~90 deg, *with no error visible anywhere* (the old project flew a whole mission
  rotated 90 deg and every internal check passed).
* **T2 — the EKF origin.** With no GPS, ArduPilot cannot derive it.  It must be set explicitly
  (`MAV_CMD_DO_SET_GLOBAL_ORIGIN`, command 611) and it must be RETRIED, because `AP_AHRS::set_origin` only
  succeeds for the ACTIVE estimator and at boot the FCU is on DCM.
* **T3 — MAVLink vision messages are not a spec you can trust by reading.** `VISION_POSITION_ESTIMATE`
  carries no frame field; `ODOMETRY` does, and demands FRD labels for NED numbers.  Signs must be pinned by
  a *move test*, not by re-derivation.
* **T4 — the truth tap declares its frame.** It does: `TAP <inst> <t_us> p_ned v_ned q(w,x,y,z body->NED)
  gyro FRD accel FRD`, SITL-native, and every conversion on our side is in `common/frames.py`.
* **T5 — RViz needs a TF tree, not a convention.** The cloud is in S, the odometry is in L, the pillars are
  in W; RViz needs the S->L edge (from the odometry) and the static W->L edge (from `Calib`).
* **T6 — a frame error can survive a small number.** A1/A2 (LIO vs truth) are computed with a fitted
  rotation, so they are *blind* to a wrong constant: the fit absorbs it.  A4 (the FCU's own estimate vs
  the truth) is not, which is why it exists, and why the round trip is printed at publish time.

## 4. The tests that must pass

Translation tests alone are NOT enough: the old project mis-plotted truth yaw against FCU yaw (Gazebo ENU
vs FCU NED, `yaw_ned = 90 - yaw_enu`), which produced a *false finding* that survived until someone
re-derived the convention.  A rotation test refuses that error immediately; a translation test cannot see
it at all.  Three classes are needed, because each pins something the others are blind to:

| class | what only it can pin |
|---|---|
| translation | the position-channel signs; catches axis swaps (needs 2+ axes, not 1) |
| rotation | attitude conventions (the 180 deg FRD/FLU roll, quaternion direction, Euler signs), the mount's angle and sign, the yaw offset, and the lever arm |
| decoupled motion | the velocity channel's frame -- translate sideways with the heading fixed |

| # | test | class | uniquely pins | status 2026-10-15 |
|---|---|---|---|---|
| T1 | +East, +North, +Up moves, 0.5 m each, heading fixed | translation | the 6 position signs; axis swaps | **covered in flight**: the square is flown in four directions and B3 arrives at each commanded corner within 100 mm (4 of 4 on the accepted sorties) |
| T2 | parked at 3 spawn headings: 0, +90, -90 deg | rotation | FRD<->FLU 180 deg roll, quaternion direction, yaw sign, and that the LIO's yaw *tracks* | **DONE**: `tools/run-convention-tests.sh`, heading error **0.000 deg** at 0/90/270, body level in W (roll/pitch 0.000 deg); the live mutation (`flip-enu`) fails with 90.000 deg / roll +180.000 deg |
| T3 | parked, tilted | rotation | the mount's sign and angle; gravity direction in the sensor frame | **DONE, parked half**: specific force in S = **45.000 deg** from the housing's +z, body pitch 0.000 deg.  The tilted half (a commanded attitude) is not flown; the mount is gated instead by the S4 surface split |
| T3b | parked, level: read the FCU's reported pitch | rotation | **that we feed the FCU the BODY attitude, not the SENSOR's** | **DONE in flight**: A4 18.6-21.1 mm horizontal against the truth over 10 flights; a 45 deg attitude error would be metres |
| T4 | crab move: heading 0 deg, translate +North | decoupled | velocity frame (`ODOMETRY`'s body-FRD velocity vs a NED velocity) | **covered in flight**: A1/A4 with `EK3_SRC1_VELXY = 6` -- the EKF's velocity source is our stream, and a wrong channel or sign runs away in seconds |
| T5 | same translation at two headings | rotation + translation | the lever arm t = (0.05, 0, 0.10) | **deferred**; the mount angle itself is gated by the S4 surface split, and the lever arm appears in `Calib.t_WL` (measured `[-0.0008, -0.0041, +0.1156]` m, i.e. the ~0.12 m origin offset) |
| T6 | in flight: reconstructed pillar bearings from the LIO's map vs the world file | shape | the mount angle *as the LIO sees it* | **half done**: `tools/sim-pillar-gate.sh` fits the pillars from the simulated sensor against the world file (p95 surface residual 0.0000 m, 37 of 37 pillars within 30 mm); the LIO's own map is not fitted back |

**Every test must also be run with an intentionally wrong conversion, and must fail.**  A test that passes
with the sign flipped has no power.  The mutation cases are written next to each test when it is built, and
two are measured: `flip-enu` (T2, above) and the S4 mount mutation (a level housing through the same gate:
floor 4.7 % vs 29.6 %).

**What S5 established, besides the tests passing:** the FCU's own attitude telemetry is *level* but its
YAW is meaningless while nothing aids it -- measured yaw 350.19 / 59.20 / 281.19 deg for commanded headings
0 / 90 / 270 (roll and pitch were correct to 0.01 deg, because gravity levels them).  That is not a frame
bug: with no GPS, no compass and no vision the EKF cannot observe yaw at all, which is exactly why the
mission feeds it the LIO.  It also means **T2's FCU half had to be re-run with an external-nav source**,
which T3b/T4 above now cover in flight.

Also measured in S5: SITL's `SR*_*` stream-rate parameters are ~0 out of the box, so a client sees only
HEARTBEAT and SYS_STATUS until it asks.  The ask is `MAV_CMD_SET_MESSAGE_INTERVAL` (511).

## 5. WHERE THE CONVERSION LIVES (the user's question: FSM? an extra MAVROS interface?)

**Neither.  One shared pure-function module, called from two places, and the steady-state loop never
crosses frames at all.**

### 5.1 The rule

* `common/frames.py` is the **only** file in the project that contains a frame conversion.  Pure
  functions, no ROS, no MAVLink, unit-tested, and each one has a mutation case.
* The **bridge implementations do no frame maths** -- with one honest exception: MAVROS performs the
  ENU->NED half itself inside its setpoint plugins, so `mavros_impl` hands MAVROS ENU-labelled numbers and
  MUST NOT also flip them.  That asymmetry is written down here because it is exactly how a double
  conversion gets introduced.
* The **FSM's steady-state loop runs entirely in the FCU's local frame** `F`.  The mission (written in W)
  is transformed into `F` **once**, when the trajectory source publishes it.  Not per setpoint.

### 5.2 The places that call it

```
common/frames.py
  w_to_l(p_w, calib)  /  l_to_w(p_l, calib)          # rotation + translation, the run's measured Calib
  l_to_f(p_l, calib)  /  f_to_l(p_f, calib)          # the FSM's steady-state conversion
  enu_ned(p)          /  ned_enu(p)                  # label flip only: (x, y, -z)
  q_flu_to_frd(q)                                    # 180 deg roll for the FRD/FLU split
  R_body_sensor()                                    # the 45 deg mount
  class Calib: R_WL, t_WL, yaw_deg(), summary()
      Calib.predicted(spawn_heading_deg=0.0, world_axes="body")   # <- the construction
      <measured every flight by tools/lio-vs-truth.py --write-calib>
```

1. **The trajectory source, at publish time** (`planner/rectangle_source.py`): the world file's square is
   read through `tools/world-info.py` and converted `W -> L` with the **same** `--calib-file` the FSM uses,
   and the `W -> L -> W` round trip is printed into `traj.log` (measured worst **0.000000 mm**), so a frame
   error is visible before take-off.
2. **The FSM at flight time** (`fsm/mission_fsm.py`): the published path arrives in L and is converted
   `L -> F` once (`legs()`), then the whole arrival/guidance loop compares the FCU's own
   `LOCAL_POSITION_NED` against those corners in **F**.
3. **External-nav publisher** (`LIO -> FCU`): the numbers need **three** fixed conversions and **one**
   constant -- but **no yaw rotation**.  Spelled out, because "no yaw" was once misread as "no conversion
   at all" (the user asked exactly this, correctly):

   | what | conversion | why it is needed |
   |---|---|---|
   | position axes | `(x, y, z)_enu -> (y, x, -z)_ned` | MAVLink position is NED |
   | attitude world | `q_enu -> q_ned` | same permutation, applied to the world side |
   | attitude body | `baselink (FLU) -> aircraft (FRD)` = 180 deg roll | ROS body is FLU, ArduPilot body is FRD |
   | origin | subtract the LIO's init position (`Calib.t_WL` = `[-0.0008, -0.0041, +0.1156]` m) | the EKF origin is the pad, the LIO's zero is the sensor |

   What is NOT here: any rotation by the angle between the LIO's world and the simulator's world.  The FCU
   adopts the LIO's yaw (`EK3_SRC1_YAW = 6`), so the LIO's frame *is* the FCU's local frame; only the
   labels differ.

   **Who performs the three conversions depends on the bridge, and it must happen exactly once:**

   | bridge | who converts ENU->NED / FLU->FRD | evidence |
   |---|---|---|
   | MAVROS | **mavros does it** -- `vision_pose_estimate.cpp:133-139` calls `transform_frame_enu_ned(position)`, `transform_orientation_baselink_aircraft(q)`; `odom.cpp:295-296` stamps `MAV_FRAME::LOCAL_FRD` / `MAV_FRAME::BODY_FRD` (which ArduPilot *requires*) | read from the mavros `ros2` branch, 2026-10-13 |
   | our pymavlink | **we do it**, in `common/frames.py`, and the bridge transports the result | by construction; this is the flown path |

   Sending already-converted numbers to MAVROS (or converting again after it) flips z and cancels the roll
   -- a silent 180-degree-class error.  Hence: one conversion, one owner.

4. **What the FSM does at steady state**: nothing that crosses a frame.

**One rule, two appliers -- stated because the constant must be the same one.**  `frames.py` owns the
conversions and the constants, but the affine step itself (`p_L = R_WL.T @ (p_W - t_WL)` and its inverse)
is applied by `planner/rectangle_source.py` (the path) and by `fsm/mission_fsm.py` (L -> F), and read back
by `tools/metrics.py` when a recording is judged.  MEASURED 2026-10-13 (`STATUS.md` row 44): when two of
them used *different* calibrations -- the PREDICTED one and the MEASURED one, 56.3 mm apart in `t` -- every
commanded corner sat 51 mm South of the intended one and the aircraft flew faithfully to the wrong place.
Both now take the same `--calib-file`, and `tools/run-mission.sh` treats a missing file as a hard error
rather than falling back to the prediction.

### 5.3 The calibration, and its self-check

`Calib` has three numbers (yaw, translation, origin) and is obtained **by measurement**, not by
assumption: a calibration flight is flown on `--nav-source truth`, and `tools/lio-vs-truth.py` fits the
rigid transform between the LIO's recording and the truth's.  Then, separately, we **predict** the same
transform from the construction argument of §0.3 and compare: measured `R_WL` yaw **+90.050 deg** against
the predicted **+90.000 deg**, and a translation whose residual rms is printed.

**The measured calibration is a FILE, and it is the one shared artifact of the run.**  It is written for
each flight as `_stage/calib.json` (`R_WL`, `t_WL`, the sample count and the residual) and read by both
appliers in §5.2.  The rule is not "each node measures its own" but "one measurement per flight, one file,
and every node that crosses frames must be given it": the prediction and the measurement differ by tens of
millimetres (56.3 mm measured 2026-10-13) and that difference flew a whole mission 51 mm off.

### 5.4 The EKF origin, and the lever arm

* Without GPS the EKF origin must be set explicitly; we set it to the **pad**, i.e. the origin of `W`, so
  that "zero" means the same thing on both sides.
* The LIO's own zero is at the **sensor** (0.05, 0, 0.10 from the body origin).  That constant is what T5
  exists to pin; it is subtracted in the external-nav publisher (`Calib.t_WL` measured
  `[-0.0008, -0.0041, +0.1156]` m), and it is the same number that appears in `lio.extrinsic.odom_robo`.
* Consequence to keep in mind when reading logs: `SET_POSITION_TARGET_LOCAL_NED`'s z is **down**, so a
  "climb to 2.0 m" is `z = -2.0` on the wire -- and the truth flies ~0.09 m below the commanded altitude
  because the F frame the FCU is commanded in sits ~0.09 m above the SITL home in z (reported as a datum in
  A4, not as a tracking error).
