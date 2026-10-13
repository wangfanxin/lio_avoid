# FRAMES — the coordinate conventions of the new stack

Written 2026-10-13 to be argued over before more code is written.  Every claim marked MEASURED is
quoted from the old project's records (which are kept in `D:\distill_self\gazebo_apm\docs`), not from
memory.  Nothing here is settled until the tests in §5 pass.

The decisions taken from this discussion are recorded in `PLAN.md` §2; the open questions in
`PLAN.md` §7.  This file owns the conventions and the tests that pin them.

## 0. HOW MANY FRAMES (the honest answer)

**Six frames EXIST.  Three of them are the ones our own logic has to hold at the same time.  Four
conversions must be written; three are fixed constants, one is the FCU's wire contract.**

The earlier version of this file said "three" and called NED/FRD "implementation details of the
bridge".  That was wrong, and the user was right to push back: NED and FRD are **wire contracts**, and
the code makes them two *different* contracts.  See §0.2.

### 0.1 The six frames that exist

| # | frame | where it appears | whose definition | can it be collapsed? |
|---|---|---|---|---|
| 1 | **FCU local NED** | `SET_POSITION_TARGET_LOCAL_NED` out, `LOCAL_POSITION_NED` / `XKF1` in | ArduPilot, origin = EKF origin | no -- it is the control interface |
| 2 | **FCU body FRD** | `ATTITUDE` telemetry, `ODOMETRY`'s `child_frame_id` | ArduPilot | no -- attitude interface |
| 3 | **our world ENU** | `worlds/pillars.yaml`, ray casting, mission geometry | us | it IS the reference |
| 4 | **body FLU** | ROS `base_link`, RViz, our sim | ROS/REP-103 | no -- the planner's body frame |
| 5 | **MID-360 housing** | the pattern CSV, the LIO's lidar+IMU input | Livox | = frame 4 + fixed mount (keep as a constant) |
| 6 | **LIO world** | the LIO's odometry output | the LIO's initialisation | merged into frame 3 by construction (see §0.3) |

"Three" is the count of frames **our logic must reason about simultaneously**: the world (3, with 6
merged into it), the body (4, with 2 as its FRD alias), and the sensor (5, a fixed offset of 4).

### 0.2 What the source says about frames on the wire (Copter-4.7.1, read 2026-10-13)

* **Setpoints** (`ArduCopter/GCS_MAVLink_Copter.cpp:1010-1013`): Copter accepts exactly four frames --
  `MAV_FRAME_LOCAL_NED`, `MAV_FRAME_LOCAL_OFFSET_NED`, `MAV_FRAME_BODY_NED`, `MAV_FRAME_BODY_OFFSET_NED`.
  Everything else is refused.  So commands are NED (absolute) or body-NED (relative).
* **`ODOMETRY` input** (`GCS_MAVLink/GCS_Common.cpp:4062-4065`):
  `if (m.frame_id != MAV_FRAME_LOCAL_FRD || m.child_frame_id != MAV_FRAME_BODY_FRD) return;` -- with the
  comment "only support local FRD frame data".  **A wrong frame id does not degrade the estimate: the
  message is dropped entirely.**  (It fails at arming rather than silently in flight, but it will look
  like "the LIO is not being fused".)
* **... and yet the numbers are consumed as NED.**  `handle_odometry` passes `m.x/m.y/m.z` straight to
  `AP_VisualOdom_MAV::handle_pose_estimate`, which applies only a scale factor and a body-offset
  correction and then calls `AP::ahrs().writeExtNavData(pos, attitude, ...)`, whose position input is
  NED.  So the enum name `LOCAL_FRD` is a *label requirement*, not the frame of the numbers we send:
  send NED numbers, label them `LOCAL_FRD`/`BODY_FRD`.  Its **velocity** is different again -- `m.vx/vy/vz`
  is body-FRD and ArduPilot rotates it by the quaternion before use.
* `VISION_POSITION_ESTIMATE` carries no frame field at all; its x/y/z are taken as NED position and its
  roll/pitch/yaw as the vehicle attitude.

**Consequence for our bridge: "send NED" is right for setpoints and for the vision position, but the
`ODOMETRY` message additionally demands FRD frame *labels*, and its velocity channel is body-FRD.**  One
message, three conventions.  This is the single most likely place for a silent sign/frame error, and it
is why the move tests in §5 are not optional.

### 0.3 Making L and W coincide on purpose (the user's question)

**Yes, and it is the single biggest simplification available.**  The mechanism is exact, not approximate:
the fork's `kf_init` aligns gravity and then *removes the vehicle's heading* (`yaw = atan2(n.y, n.x)` of
the body x-axis, then `rot = g_lidar_robo_yaw * R_yaw_inv * init_rot`), so **L's x-axis is the body's
forward axis at the instant the LIO initialises**.  If the vehicle is parked on the pad facing a known
direction when the LIO starts, the whole L <- W relationship is decided by one number we choose at spawn
time.  Two consequences follow, and they pull in opposite directions:

| spawn heading at LIO init | L relative to W | the FCU's local NED relative to W |
|---|---|---|
| **facing North** (W's +y) | L's x is W's North, so the map W -> L is a -90 deg yaw and the map L -> W is +90 deg | **the standard permutation** `(x_ned, y_ned, z_ned) = (N, E, -U)`; "North" means North |
| facing East (W's +x) | **L = W exactly** (axes identical, both maps identity) | a 90 deg rotation: the FCU's "North" is W's East |

Direction convention, because this is exactly where a sign hides: `R_WL` maps **L coordinates to W**
(`v_W = R_WL @ v_L`), so `R_WL[:,0]` is L's x-axis written in W.  Facing North that column is
`(0, 1, 0)` and `yaw(R_WL) = +90 deg`; the inverse map `R_LW = R_WL.T` has yaw -90 deg.  Both numbers
appear in this project's notes, so `frames.py` says which one every function returns, and
`Calib.yaw_deg()` reports `R_WL`'s yaw (measured +90.000 deg for a North spawn, `common/frames.py`
self-test).

**Note added 2026-10-13 (the module's own test caught it):** "North" here is a **compass** heading
(0 = North), while a rotation about W's z is an **ENU** yaw (0 = East, anticlockwise).  They differ by
90 deg and the same word "yaw" is used for both in older notes.  `frames.py` therefore has
`R_WB_from_heading(heading_deg)` for the compass reading and `R_z(angle_rad)` for the ENU one, and
never takes one where the other belongs.

They cannot both hold: with `EK3_SRC1_YAW=6` the EKF **adopts the LIO's yaw**, and the LIO's yaw zero is
the body's forward axis at init.  So the FCU's "North" *is* the body's init heading, and L's x-axis is the
same direction; making L = W forces the FCU's North to be W's East.

**Recommendation: spawn facing North.**  What must be provable is the **FCU link** -- that is where the
mission is sent and where a silent rotation flies the square rotated while every internal check passes.
Facing North makes that link the textbook permutation with no yaw parameter anywhere in the mission path,
and leaves L offset from W by a constant -90 deg that lives in one line of `frames.py` and is verified
once.  The planner does not care: its map and its state are both in L.

With that choice the mission transform collapses to a swap and a sign flip:

```
(x_ned, y_ned, z_ned) = (N_rel, E_rel, -U_rel)      # relative to the EKF origin (the pad)
```

No rotation, nothing to calibrate, and the "which frame is this in" class of errors loses its hiding place.

**What is left, and what each part costs:**

1. **Origin.**  L's origin is the LIO's init position, which the fork sets to the lever arm
   `g_odom_robo.t_ = (0.05, 0, 0.10)` -- about 0.3 m above the pad.  Either (a) subtract the constant in
   one place (no LIO change; my preference to start), or (b) add a **default-off** parameter to `kf_init`
   so the *published body odometry* starts at exactly (0,0,0).  (b) is ~5 lines and must stay default-off,
   so the upstream A/B in the LIO bake-off remains a real A/B.
2. **The spawn condition is declared and checked, not assumed.**  `worlds/pillars.yaml` therefore carries
   an explicit `spawn:` block (position + yaw), and the FSM asserts from the truth tap that the vehicle
   really was parked, level and at that heading when the LIO initialised.  If it was not, the flight does
   not start: coincidence makes the chain simple *and* makes it depend on one initial condition, so that
   condition gets a check instead of a hope.
3. **What coincidence does NOT fix.**  It is a frame statement, not an estimator statement: the 2.5 deg
   pitch of the LIO's estimate that round 7 found (and that A2's 31-37 mm is made of) is unaffected, and
   neither is any noise or jitter question.

### 0.45 Where B (body FLU) is used, and why it cannot be merged -- but can be demoted

**B is used in exactly four places:**

1. **The parent of the sensor mount**: `S = B` rotated by the 45 deg nose-down mount and offset by
   (0.05, 0, 0.10).  Without B there is nothing to hang the housing on.  Sign, stated once: a
   nose-down mount means the housing's forward axis lies **below** the body's, i.e. in body
   coordinates `x_S = (cos 45, 0, -sin 45)` and `R_BS = R_y(+45)` -- the housing's *up* axis tilts
   forward.  The flight stack states the same mount as `odom_robo` pitch +45 with its rpy applied
   inverted, so the two documents disagree in words and agree in physics; `R_body_sensor()` encodes
   the physics and tests T5/T6 measure it against the world instead of trusting either wording.
   (`common/frames.py` self-test also settles the second half: with L's x = the body's forward axis,
   the mount does not change L's *heading*, only the elevation of its axes.)

   **MEASURED 2026-10-13, and worth knowing before reading any scan picture:** tilting a *dome* does
   not tilt a fan.  With the housing 45 deg nose-down the pattern's elevation band (-7.16..+52.16 deg
   about the housing) becomes a *cone about a tilted axis*, so the world-elevation coverage runs from
   about -52 deg (forward-down) to about +83 deg (backward-up) -- which is exactly why this mount keeps
   the floor AND the ceiling in view.  Measured split at the pad: **floor 29.6 %, ceiling 16.8 %,
   walls 8.7 %, pillars 44.9 %**; a level housing gives **floor 4.7 %, ceiling 7.1 %, pillars 74.3 %**.
   Those two lines are the mount test (an earlier check that demanded a ceiling-dominated level case
   was based on a 2D argument and was simply wrong; a 2D band would have made the max world elevation
   +7 deg and the ceiling unreachable, yet the ceiling is measured at 16.8 %).
2. **The published odometry frame**: `/odom` (after the LIO applies `odom_robo`) is the body pose -- what
   the planner consumes as `base_link` and what RViz draws.  `/lio/odom` is the same state in S.
3. **The FCU's external-nav attitude**: ArduPilot wants the *body* attitude; its own body convention is
   FRD, which is B under a fixed 180 deg roll.  (Mission A sets `VISO_POS_X/Y/Z = 0`, i.e. we send the body
   origin, not the sensor's.)
4. **Any body-relative command** (`MAV_FRAME_BODY_NED` / `BODY_OFFSET_NED`) and every footprint or
   collision check, all of which are defined about the body origin.

**It cannot be merged into W or L**, and the reason is not bookkeeping: W and L are *fixed* frames, B is
the *moving* one.  Its pose in L is exactly what the LIO measures; merging would assert the vehicle never
rotates.

**But it can be demoted, and that is worth doing.**  B is not an independent frame with its own
conventions: it is a *fixed offset from S* and *the frame the odometry is published in*.  So:

| frame | who ever mentions it | why |
|---|---|---|
| **W** | the mission file, the FSM at set-up, RViz | the world geometry lives here |
| **L** | LIO output, the planner, the FSM loop, the metrics | the map and the state live here |
| **B** | `/odom`, `base_link`, the external-nav message, body-relative commands | one rigid offset from S, one 180 deg roll from the FCU's body |
| **S** | only `sim/lidar_imu` (publishes here) and the LIO (consumes here, and reports `/lio/odom`) | an implementation detail of the sensor + LIO pair |

So the chain reads: **sim generates in S -> the LIO consumes S and publishes B (in L) -> everything else
speaks B in L, or W at mission set-up.**  Only two files in the project ever name S, and one number (the
mount) connects S to B.

**The failure this demotion must not hide**: feeding the FCU the *sensor* attitude instead of the *body*
attitude.  The vehicle would look pitched 45 deg to its own controller, which would then try to correct
it.  The parked attitude test (section 5, T2/T3) is where that is caught, in its sharp form: **on the pad
the FCU's reported pitch must be ~0, never ~45 deg.**

### 0.4 The four conversions

| # | conversion | nature | pinned by |
|---|---|---|---|
| 1 | `world ENU <-> FCU local NED` | fixed permutation (+ the FCU frame's yaw = the LIO's) | +East/+North/+Up move tests |
| 2 | `body FLU <-> FRD` | fixed 180 deg roll | parked attitude test |
| 3 | `body <-> housing` | fixed 45 deg mount + (0.05, 0, 0.10) | parked scan: 23.5 % floor / 16.6 % ceiling at 0.5 m |
| 4 | `LIO world <-> world` | known constant (fix the spawn heading) | assert the heading at LIO init from the truth tap |

## 1. The frames that exist

| # | frame | axes | origin | produced by | consumed by |
|---|---|---|---|---|---|
| F1 | SITL / ArduPilot local **NED** | x North, y East, z Down | EKF origin — **must be set explicitly** (no GPS in mission A) | ArduPilot | GUIDED setpoints, XKF1 comparison |
| F2 | ArduPilot **body FRD** | x fwd, y right, z down | vehicle CG | ArduPilot | logged attitude |
| F3 | **world.yaml / sensor-sim world** | x East, y North, z Up | the pad | our sim | ray casting, mission geometry, RViz |
| F4 | our sim **body FLU** | x fwd, y left, z up | body origin | our sim | the mount transform |
| F5 | **MID-360 housing** (lidar AND IMU) | x fwd, y left, z up, mounted **45 deg nose-down** | housing centre | our sim (cloud + IMU) | the LIO |
| F6 | **LIO world** | gravity-aligned, z up, **yaw arbitrary per run** | where the LIO initialised | Super-LIO / FAST-LIO | the EKF |
| F7 | the FCU's local frame **as the EKF sees it** | NED, **but yaw-locked to F6** | EKF origin | ArduPilot | GUIDED setpoints |
| F8 | ROS / planner / RViz | ENU + FLU (REP-103/105) | `map` / `odom` | us | ego-planner, RViz |

## 2. What is already MEASURED (do not re-litigate)

* **F5's azimuth convention.** The bridge that fed the LIO published the cloud untransformed, as
  `point.x = radial_xy * cos(h)`, `point.y = radial_xy * sin(h)` — i.e. azimuth measured from +x toward
  +y, counter-clockwise seen from above.  That is the convention `sim/raycast.py` uses, and the real
  `scan_mode/mid360.csv` agrees with it (zenith from +z; MEASURED here: reading that column as
  elevation puts the fan in the wrong half of the sky — the file's own numbers are elevation
  -7.16..+52.16 deg after conversion).
* **F5 carries BOTH sensors.** `livox_ros_driver2` sets `frame_id = livox_frame` for the cloud and for
  the IMU, and copies gyro/accel with **no rotation** (`lddc.cpp:490-495`).  So on hardware, and in our
  new sim, the lidar and the IMU are one rigid housing: `lio.extrinsic.lidar_imu` is **identity**.
* **The mount is NOT in that extrinsic.** The 45 deg nose-down mount belongs to lidar->body
  (`lio.extrinsic.odom_robo`), whose documented sign convention is that the transform actually applied
  is the INVERSE of the stated rpy: to get `Ry(-45)` applied, pass pitch `+45`.  Encoding the mount in
  `lidar_imu` "would inject a spurious 45 deg on the real drone" (the old config's own words).
* **F6's yaw is arbitrary, and that is by design.** The LIO's world is gravity-aligned only; its yaw is
  whatever heading the vehicle had at initialisation.  MEASURED across five flights, the rotation
  between the LIO's world and the FCU's: -89.7, +115.7, -88.0, -11.3, -87.9 deg.  "A fixed transform
  between two frames cannot do that."
* **Therefore F7 is rotated relative to F3 by an unknown per-run angle.** With `EK3_SRC1_YAW=6` the EKF
  *adopts* F6's yaw, so the FCU's local NED is F6, not the simulator's NED.  The old project's
  conclusion: **no frame alignment is needed anywhere in the chain** — the naive +-180 deg
  "alignment" they once had was the bug, not the fix.
* **The 45 deg mount is SETTLED (user decision, 2026-10-13).** MEASURED before deciding: at 0.5 m
  sensor height the 45 deg nose-down fan returns 23.5 % floor hits and 16.6 % ceiling; a level mount
  would put the fan at -7..+52 deg about the body, i.e. ceiling only and essentially **no floor**
  (a -7 deg ray reaches the floor only at 20 m), and the vertical channel would lose its best
  constraint -- which matters because mission A takes its height from the LIO (`EK3_SRC1_POSZ=6`).

## 3. The traps

* **T1 — mission geometry lives in the wrong frame if you are not careful.** We define the square in
  F3 (world.yaml).  The GUIDED setpoints we send are in F7.  Without a measured offset the square comes
  out rotated by an arbitrary angle, *with no error visible anywhere* (the old project flew a whole
  mission rotated 90 deg and every internal check passed).
* **T2 — the EKF origin.** With no GPS, ArduPilot cannot derive it.  It must be set explicitly
  (`SET_GPS_GLOBAL_ORIGIN`) — the old project never exercised this because it always had GPS.
* **T3 — MAVLink vision messages are not a spec you can trust by reading.** `VISION_POSITION_ESTIMATE`
  carries no frame field; `ODOMETRY` does.  The old adapter carried a hand-written mapping
  (`E_enu = -y_lio`) whose sign nobody re-derived.  Signs must be pinned by a *move test*.
* **T4 — the truth tap must declare its frame.** Mine currently says "NED/ENU per the tap's declared
  frame", which is not a decision, it is a placeholder.  See P1.
* **T5 — RViz needs a TF tree, not a convention.** If the cloud is in F5 and the planner's odometry is
  in F6, RViz needs F5->F6 (from odometry) and, if we want to see F3 geometry, a static F3->F6 measured
  at init.

## 4. What I propose

* **P1 (truth tap):** emit SITL-native values and declare them in the header —
  `TRUTH <t> <p_ned xyz> <v_ned xyz> <q w x y z, body->NED> <w frd> <a frd> <origin>`.  Zero conversion
  inside SITL (that is where sign errors are born); one Python module `sim/frames.py` does every
  conversion, with unit tests.  Alternative if you prefer: emit ENU/FLU so the ROS side needs nothing —
  but then SITL carries the conversion.
* **P2 (sensor sim):** cloud and IMU published in **F5**, mount applied in the ray directions (already
  the case), IMU in F5, `lidar_imu = identity`.
* **P3 (mission):** geometry defined in **F3**, converted to F7 at runtime with a **measured** yaw
  offset, and the offset re-measured every flight (it changes every run).
* **P4 (mount):** keep 45 deg nose-down.
* **P5 (metrics):** the analysis keeps fitting a rotation and keeps *printing* which fit it used
  (round 7 already does: yaw+2D for A1/A2/A4, full 3D where the tilt matters).

## 5. The tests that must pass before any mission

Translation tests alone are NOT enough, and this is not a theoretical worry: the old project mis-plotted
truth yaw against FCU yaw (Gazebo ENU vs FCU NED, `yaw_ned = 90 - yaw_enu`), which produced a *false
finding* ("truth yaw runs out to 75 deg") that survived until someone re-derived the convention.  A
rotation test refuses that error immediately; a translation test cannot see it at all.

Three classes are needed, because each pins something the others are blind to:

| class | what only it can pin |
|---|---|
| translation | the position-channel signs; catches axis swaps (needs 2+ axes, not 1) |
| rotation | attitude conventions (the 180 deg FRD/FLU roll, quaternion direction, Euler signs), the mount's angle and sign, the yaw offset, and the lever arm (a rotation-dependent position offset is invisible at a fixed attitude) |
| decoupled motion | the velocity channel's frame -- translate sideways with the heading fixed, otherwise "body-FRD velocity" and "world-NED velocity" look identical |

| # | test | class | uniquely pins | cost |
|---|---|---|---|---|
| T1 | +East, +North, +Up moves, 0.5 m each, heading fixed | translation | the 6 position signs; axis swaps | one short scripted motion (or one flight) |
| T2 | parked at 3 spawn headings: 0, +90, -90 deg | rotation | FRD<->FLU 180 deg roll, quaternion direction, yaw sign, and that the LIO's yaw *tracks* rather than sitting at a constant | free: restart the sim ~1 min each, no arming, no flight |
| T3 | parked, tilted (pitch/roll 10 deg) | rotation | the mount's sign and angle; gravity direction in the sensor frame (round 7 already measured 45.001 deg parked) | free-ish: spawn on a tilted pad or command attitude while parked |
| T3b | parked, level: read the FCU's reported pitch | rotation | **that we feed the FCU the BODY attitude, not the SENSOR's.**  A 45 deg reading here means the housing attitude reached the external-nav message.  Expected: 0 deg within a degree | free: parked, no arming |
| T4 | crab move: heading 0 deg, translate +North | decoupled | velocity frame (ODOMETRY's body-FRD velocity vs a NED velocity) | one short flight |
| T5 | same translation at two headings (0 and +90 deg) | rotation + translation | the lever arm t = (0.05, 0, 0.10) | one parked rotation + one move |
| T6 | in flight: reconstructed pillar bearings from the LIO's map vs world.yaml | shape | the mount angle *as the LIO sees it* (a wrong mount tilts the map) | one flight |

Note that T2/T3/T5 need no flight at all -- the sim can be restarted with a different spawn heading and
the whole comparison done parked.  Only T1, T4 and T6 need the vehicle to move.

**Every test must also be run with an intentionally wrong conversion, and must fail.**  A test that
passes with the sign flipped has no power, and this project's discipline has been to A/B every claim
against a known-bad configuration.  The mutation cases are written next to each test when it is built.

## 7. WHERE THE CONVERSION LIVES (the user's question: FSM? an extra MAVROS interface?)

**Neither.  One shared pure-function module, called from two places, and the steady-state loop never
crosses frames at all.**

### 7.1 The rule

* `frames.py` is the **only** file in the project that contains a frame conversion.  Pure functions,
  no ROS, no MAVLink, unit-tested, and each one has a mutation case (flip the sign on purpose and the
  test must fail).
* The **bridge implementations do no frame maths** -- with one honest exception: MAVROS performs the
  ENU->NED half itself inside its setpoint plugins, so `mavros_impl` hands MAVROS ENU-labelled numbers
  and MUST NOT also flip them.  That asymmetry is written down here because it is exactly how a double
  conversion gets introduced.
* The **FSM's steady-state loop runs entirely in the FCU's local frame** `F`.  The mission (which is
  written in world ENU, `W`) is transformed into `F` **once, at flight start**.  Not per setpoint.
  Fewer crossings, fewer chances to be wrong.

### 7.2 The two places that call it

```
frames.py
  w_to_f(p_w, calib)  /  f_to_w(p_f, calib)          # rotation + translation, known constant calib
  f_to_ned(p_f)       /  ned_to_f(p_ned)             # label flip only: (x, y, -z)
  q_w_to_f(q_w)       /  q_f_to_frd(q_f)             # 180 deg roll for the FRD/FLU split
  class Calib: yaw, t, origin
      Calib.measure(tap_truth, fcu_local_pos, lio_pos_at_init)   # <- measured every flight
      Calib.check_against_construction()                          # <- predicted from the spawn heading
```

1. **FSM at start-up**: `mission_f = [w_to_f(c, calib) for c in mission_w]`, then the whole arrival /
   guidance loop compares the FCU's own `LOCAL_POSITION_NED` (via `ned_to_f`) against `mission_f`.
   The planner, if it is running, works in the LIO's frame, which *is* `F`.
2. **External-nav publisher** (`LIO -> FCU`): the numbers need **three** fixed conversions and **one**
   constant -- but **no yaw rotation**.  Spelled out, because "no yaw" was once misread as "no
   conversion at all" (the user asked exactly this, correctly):

   | what | conversion | why it is needed |
   |---|---|---|
   | position axes | `(x, y, z)_enu -> (y, x, -z)_ned` | MAVLink position is NED |
   | attitude world | `q_enu -> q_ned` | same permutation, applied to the world side |
   | attitude body | `baselink (FLU) -> aircraft (FRD)` = 180 deg roll | ROS body is FLU, ArduPilot body is FRD |
   | origin | subtract the LIO's init position (0.05, 0, 0.10 from the body origin) | the EKF origin is the pad, the LIO's zero is the sensor |

   What is NOT here: any rotation by the angle between the LIO's world and the simulator's world.  The
   FCU adopts the LIO's yaw (`EK3_SRC1_YAW=6`), so the LIO's frame *is* the FCU's local frame; only the
   labels differ.  That is why the old project concluded "no frame alignment anywhere in the chain" --
   and why the +-180 deg aligner they once had was the bug rather than the fix.

   **Who performs those three conversions depends on the bridge, and it must happen exactly once:**

   | bridge | who converts ENU->NED / FLU->FRD | evidence |
   |---|---|---|
   | MAVROS | **mavros does it** -- `vision_pose_estimate.cpp:133-139` calls `transform_frame_enu_ned(position)`, `transform_orientation_baselink_aircraft(q)` and `transform_orientation_enu_ned(...)`; `odom.cpp:295-296` stamps `MAV_FRAME::LOCAL_FRD` / `MAV_FRAME::BODY_FRD` (which ArduPilot *requires*, else it drops the message) | read from the mavros `ros2` branch, 2026-10-13 |
   | our pymavlink | **we do it**, in `frames.py`, and the bridge transports the result | by construction |

   Sending already-converted numbers to MAVROS (or converting again after it) flips z and cancels the
   roll -- a silent 180-degree-class error.  Hence: one conversion, one owner, and the same move and
   rotation tests run for **both** bridge settings.

3. **What the FSM does at steady state**: nothing that crosses a frame.  The mission is transformed once
   (item 1) and the loop compares the FCU's own `LOCAL_POSITION_NED`, converted with `ned_to_f`, against
   `mission_f`.

### 7.3 The calibration, and its self-check

`Calib` has three numbers (yaw, translation, origin) and is obtained **by measurement**, not by
assumption: at flight start we hold the truth tap's pose (in `W`) and the FCU's own reported local
position / attitude at the same instant, and solve for the rigid transform between them.

Then, separately, we **predict** the same transform from the construction argument of §0.3 (the LIO's
world x-axis is the body's forward axis at initialisation, the vehicle was parked on the pad at a known
heading) and compare.  If measurement and prediction disagree by more than a few degrees, **something
is wrong with a convention, and the flight does not start.**  This is the cheapest possible detector for
the whole class of errors discussed in this file, and it costs nothing.

### 7.4 The EKF origin, and the lever arm

* Without GPS the EKF origin must be set explicitly; we set it to the **pad**, i.e. the origin of `W`,
  so that "zero" means the same thing on both sides.
* The LIO's own zero is at the **sensor** (0.05, 0, 0.10 from the body origin).  That constant is what
  T5 exists to pin; it is subtracted in the external-nav publisher, and it is the same number that
  appears in `lio.extrinsic.odom_robo`.
* Consequence to keep in mind when reading logs: `SET_POSITION_TARGET_LOCAL_NED`'s z is **down**, so a
  "climb to 2.5 m" is `z = -2.5` on the wire.
