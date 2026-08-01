# Vision-based SFP insertion

Design for the ROS 2 module in `ros2/aic_insertion/` that inserts the SFP plug
tip into NIC port 0 using **camera images, TF, joint states and the wrist F/T
sensor only**. The `/aic/cheat/*` topics are used for *validation exclusively* —
the runtime pipeline never subscribes to them.

This document records the system analysis the design rests on, the candidate
approaches that were compared, the chosen architecture, the validation strategy
and the implementation plan.

---

## 1. What exists — system analysis

### 1.1 Data flow

```
Isaac Sim (OmniGraph action graph /World/aic/ROS2_Graph, ticks ~10 Hz)
  ├─ pub /clock                                  rosgraph_msgs/Clock
  ├─ pub /joint_states        (6 arm joints)     sensor_msgs/JointState
  ├─ sub /joint_command  ──►  IsaacArticulationController (PD 2000/100, position)
  ├─ pub /tf                  (robot links only) tf2_msgs/TFMessage, parent "World"
  ├─ pub /aic/{center,left,right}_camera/rgb     sensor_msgs/Image   (224×224)
  ├─ pub /aic/{center,left,right}_camera/camera_info                 (fx=fy=240.3, cx=cy=112)
  ├─ pub /wrist_ft/wrench     (ati_tool_link)    geometry_msgs/WrenchStamped
  └─ pub /aic/cheat/{entrance,seat,tip}_pose, insertion_fraction     ── validation only
```

Everything ticks off one `OnPlaybackTick` gated by the three camera render
products, so all topics run at the render rate (~10 Hz). `/joint_command` is a
position target; the drives are stiff (stiffness 2000, damping 100), so
commands must be ramped (see `scripts/move_arm.py`).

### 1.2 Coordinate frames and conventions

Quaternions are **wxyz** inside the specs/Isaac code and **xyzw fields** in ROS
messages (the bridge wires the components explicitly, so nothing is ambiguous
on the wire). All poses are metres, world is Z-up, TF root frame is `World`.

TF publishes **every rigid body under `aic_unified_robot`** flat against
`World`:

```
World ── base_link … wrist_3_link ── flange ── tool0
                                                ├─ cam_mount ─ {center,left,right}_camera_{camera,sensor,optical}
                                                └─ ati_base_link ─ ati_tool_link ─ gripper_hande_base_link ─ gripper_tcp
gripper_hande_finger_link_r ══weld══ lc_plug ── sfp_module ── sfp_tip_link      (cable branch: NOT in TF)
```

Facts that the design leans on (all read out of the vendored USD/specs, i.e.
model knowledge a real deployment would take from CAD/URDF):

- **The SFP tip is rigid w.r.t. the gripper.** The chain
  finger→`lc_plug`→`sfp_module`→`sfp_tip_link` is all fixed joints; the
  compliant rope hangs off the plug but does not move the tip. Constant
  transform: `T(gripper_tcp→sfp_tip_link) = pos(-0.00431, -0.01746, 0.05677),
  quat wxyz(0, 0, 0.98901, 0.14782)`.
  **This must be read from the joint constraint frames, not from the authored
  xforms.** The two disagree by 90° about the tool axis, and PhysX snaps the
  assembly onto the joints at Play — so the stage's authored poses describe a
  configuration that never exists at runtime. Taking the authored value put
  every commanded TCP pose 84 mm from the truth, which cost a full debugging
  cycle (§8). `verify_specs.py` now derives it from the joints and the value
  is confirmed live against `/tf`.
- **`sfp_tip_link_robot` in `/tf` is junk.** It is a rigid body with *no
  joint* attaching it — it free-falls on Play. The physical tip
  (`cable/sfp_module/sfp_tip_link`) is not in `/tf`. The tip pose is therefore
  computed as `T(World→gripper_tcp) ∘ T(gripper_tcp→tip)`.
- **Cameras are wrist-mounted (eye-in-hand), all three on one mount** bolted
  to `tool0`: center between the fingers' plane, left/right ±94 mm to the
  sides, all converging on the tip ~0.28 m in front of the optics. Their
  `*_camera_optical` frames are rigid bodies → published in `/tf`, and they
  follow the **ROS optical convention** (+Z forward, +Y down); the USD camera
  prim child holds the 180°-about-X flip, and images carry
  `frame_id=<name>_camera_optical`. Back-projection with `camera_info` + TF
  therefore works with the textbook pinhole model, no extra flips.
- **Port geometry** (in `nic_card_link`, from `nic_card.usd`): seat frame
  `sfp_port_0_link` at (0.01295, −0.03157, 0.00501), entrance frame 45.8 mm
  further out at (0.01295, −0.07737, 0.00501), both identity-oriented →
  insertion axis is card +Y (`insertion_axis_local=(0,1,0)`). The entrance
  *opening* is a 13.96 × 8.8 mm rectangle whose four corners exist as named
  Xforms (`sfp_port_0_{front,back}_{left,right}`). A second identical port
  (`sfp_port_1`) sits 23.2 mm away — a mandatory disambiguation case, and a
  free extra constraint when both are detected.
- **The card stands with the ports opening upward.** The target slot rotation
  maps card X→−X, Y→−Z, Z→−Y (world). Insertion is **straight down**
  (world −Z), the entrance opening is a **horizontal** rectangle, and because
  the randomization moves the board only in x/y/yaw and the card only along
  the board-frame snap grid, the entrance plane height is a **fixture
  constant**: `z = 0.0743 + 0.07737 = 0.15167 m`, roll = pitch = 0. The only
  unknown card DOFs are **(x, y, yaw)** plus which snap slot it is in.
- **Goal convention** (mirrors `InsertionGoalCommand` and the cheat topics):
  the published entrance/seat goals are the port frames composed with the
  fixed EEF offset `eef_pose_in_port_frame = pos(0, 0, 0.0013),
  quat wxyz(√.5, √.5, 0, 0)` — i.e. *where `sfp_tip_link` must be*, not where
  the port frame is. The module reproduces exactly this composition so its
  goals are directly comparable to the cheat topics.

### 1.3 The cheat routine (studied, not reused)

`etf_robotics_aic/scripts/il/path_planners/port_insertion.py` drives three
open-loop phases from the privileged goals — APPROACH (to a standoff point
`entrance − 0.05 m·axis`, orientation held), ALIGN (translate the last lateral
bit while SLERPing to the entrance orientation), INSERT (down the axis to the
seat at 10 % speed). Segments are rest-to-rest quintics
`s(τ)=10τ³−15τ⁴+6τ⁵`, per-phase speed scales 0.6/0.8/0.1, and endpoints are
shifted from EEF (tip) frame to TCP frame with the constant `tcp_in_eef`
offset. Success (upstream termination spec): tip within **3 mm / 4°** of the
seat goal held **0.5 s**. Progress metric: fraction along entrance→seat,
gated to zero if the tip strays > **2 mm** off-axis.

The routine's *shape* is preserved (standoff, phase structure, quintic
timing, tip→TCP shift, thresholds); its *inputs* are replaced by perception.

### 1.4 What the randomization can change (the perception envelope)

Board: x, y ∈ ±0.04 m, yaw ∈ ±0.35 rad (±20°). Card: carried by the board,
plus a slide of 0…0.12 m in 0.04 m snap steps along board-local +Y. Dome
light: intensity 1500–3500, colour tint 0.5–1.0 grey. Nothing else moves;
z / roll / pitch of every fixture are constant.

---

## 2. Problem statement and information budget

Estimate `T(World → nic_card_link)` — effectively (x, y, yaw) — accurately
enough to place a 13.96 mm-wide plug into a 13.96 mm-wide, 8.8 mm-tall opening
(sub-millimetre lateral, few-degree yaw), then execute a guarded vertical
insertion of 45.8 mm.

Allowed inputs: RGB ×3 + `camera_info`, `/tf` (robot proprioception),
`/joint_states`, `/wrist_ft/wrench`, and model constants (port CAD, fixture
height, tool geometry, DR envelope as priors). Forbidden at runtime:
`/aic/cheat/*`, reading sim state directly.

---

## 3. Candidate approaches

### A. Classical geometric pipeline (chosen)

Detect the dark port openings as quadrilaterals, refine corners to sub-pixel,
back-project each corner ray onto the **known horizontal entrance plane**
(z = 0.15167), and fit the known one-or-two-rectangle model to the resulting
2D points → (x, y, yaw) per camera, fused across cameras and time.

- ➕ Deterministic, interpretable, ~ms per frame on CPU, zero new
  dependencies (OpenCV 4.6 + NumPy are already on the machine).
- ➕ The ray∩plane step removes the depth/tilt ambiguity that makes single-view
  PnP of small rectangles fragile; residuals against the rigid two-port model
  give a built-in quality gate.
- ➕ Accuracy budget: at ~0.30 m range a 224-px image gives ≈1.25 mm/px —
  marginal; at 448 px ≈0.62 mm/px, and sub-pixel corners (σ≈0.2–0.3 px) ×
  8–16 corners × 3 views × temporal averaging lands well under the ±0.5 mm
  the funnel needs (measured, not assumed — see §6).
- ➖ Hand-tuned robustness to the lighting DR; mitigated by adaptive
  thresholding inside prior-predicted ROIs and by strict geometric gates
  (size, aspect, plane height, port-pair spacing) rather than appearance.
- ➖ Specific to this port type — acceptable: the geometry comes from the spec,
  and the detector sits behind an interface.

### B. Zero-shot vision foundation model for detection

OWL-ViT / GroundingDINO ("SFP cage opening") or SAM2 for segmentation, then
the same geometric refinement as A (mm-accuracy always ends geometric).

- ➕ Robust open-set detection, no threshold tuning, graceful under appearance
  shift.
- ➖ Torch is not installed; ~2.5 GB of dependencies competing for the single
  RTX 3060 Ti (8 GB) that Isaac Sim is already saturating.
- ➖ 100 ms–1 s latency per camera per frame — fine for one-shot acquisition,
  unusable in the refine-while-approaching loop.
- ➖ 224–448 px renders of an uncommon industrial part are a weak zero-shot
  target; failure modes are silent and version-dependent.
- Verdict: only replaces the *coarse* stage, which is exactly the stage the
  strong task priors already make easy. Not worth the cost here; the
  `PortDetector` interface keeps the slot open.

### C. Task-specific learned detector (the sim-2-real path)

Small UNet/keypoint head trained on auto-labelled DR renders — the repo can
generate labels for free (randomize layout, project the USD corner frames
through `camera_info` + TF). Best robustness-per-millisecond and the right
long-term answer for transfer to real cameras; requires a training loop and
torch. Deferred: the module ships the detector interface and the geometry
needed to auto-label, so this drops in later without touching the estimator
or controller.

**Decision: A**, structured so B or C can replace the detector stage without
touching anything downstream.

---

## 4. Architecture

Two nodes (perception is reusable without the controller), one small kinematics
library, all in one ament package `ros2/aic_insertion/`.

```
                       ┌────────────────────────────────────────────┐
 /aic/*_camera/rgb ──► │ port_perception_node                       │
 /aic/*_camera/       │  per camera: predict ROI → dark-quad detect │
     camera_info  ──► │  → sub-pixel corners → ray ∩ entrance plane │──► /aic/insertion/port_pose
 /tf (optical    ──►  │  → 2-rect model fit (x,y,yaw) → gates       │       (PoseWithCovarianceStamped, card frame)
      frames)          │  fuse cameras ▸ robust average ▸ EMA        │──► /aic/insertion/entrance_pose, seat_pose
                       └────────────────────────────────────────────┘       (PoseStamped, tip goals, cheat-compatible)
                                                                    └─► /aic/insertion/debug/*_overlay (Image)

                       ┌────────────────────────────────────────────┐
 /joint_states  ─────► │ insertion_node — state machine             │
 /tf ────────────────► │  HOME→ACQUIRE→APPROACH→ALIGN→REFINE→       │──► /joint_command (ramped positions)
 port estimate ──────► │  INSERT(force-guarded)→SEATED / RETRY      │──► /aic/insertion/status (String)
 /wrist_ft/wrench ───► │  Cartesian quintic segments +               │
                       │  closed-loop DLS differential IK (UR5e)    │
                       └────────────────────────────────────────────┘
```

Design points:

- **Frames**: the estimator's state is the 3-DOF card pose; entrance/seat tip
  goals are derived by the same composition the cheat pipeline uses, so
  `/aic/insertion/*_pose` and `/aic/cheat/*_pose` are directly diffable.
- **Control** mirrors upstream: differential IK (damped least squares,
  λ = 0.01), per-phase speed caps 0.6/0.8/0.1 of a configured v_max, quintic
  time-scaling, EEF→TCP endpoint shift. Closed-loop: FK from `/joint_states`
  is cross-checked against `/tf` `gripper_tcp` every cycle (hard abort on
  divergence — catches kinematic-model drift).
- **Kinematics** are extracted from the USD joint frames (localPos0/Rot0,
  localPos1/Rot1, axis per revolute joint) into a spec dataclass — no DH
  guessing, exact by construction, robot-agnostic in shape. `World→base_link`
  comes from `/tf` at startup (the base is static).
- **Force guard**: wrench is tared at the standoff; INSERT aborts on force
  spikes, backs off 20 mm, re-acquires, and retries with a small lateral
  spiral (≤1 mm) — the classic peg-in-hole recovery, bounded to 3 attempts.
- **Success** is declared from proprioception: tip within 3 mm / 4° of the
  internal seat goal for 0.5 s (upstream thresholds), never from cheat data.
- The package vendors its constants (port model, tool transform, priors) in
  `specs.py`, mirroring how this repo vendored `asset_specs` from IsaacLab; a
  `verify_specs.py` script re-derives every number from `aic_sim.specs` + the
  USDs and fails loudly on drift.

### Simulation changes (all justified, all optional)

1. **`--camera-res N` flag on `run_sim.py`** (default 224 = today's
   behaviour). 224×224 was chosen upstream as an RL policy input size, not a
   sensor property; at 0.3 m it yields 1.25 mm/px, which taxes the accuracy
   budget for a 0.4 mm-clearance task. Real deployments would use ≥720p
   cameras. The flag scales the render products (intrinsics in `camera_info`
   scale automatically); perception runs are documented at 448.
   Implementation: a `dataclasses.replace` override of the camera frames at
   bridge-build time — `aic_sim/specs/robots.py` stays a straight mirror of
   upstream, and the divergence lives where the README says it belongs.
2. **Nothing else.** No depth, no segmentation, no fiducials, no extra TF:
   the plane prior replaces depth; fiducials would change the task; the card
   pose in TF would be the answer itself.

---

## 5. Why this design (summary of reasoning)

1. **Exploit structure before machinery.** The task has an unusually strong
   prior — 3 unknown DOFs, a horizontal opening plane at an exactly known
   height, an exactly known rectangle pair. Ray∩plane + rigid 2-rect fit is
   the minimal estimator that consumes all of it; anything heavier adds
   failure modes, not accuracy.
2. **Accuracy comes from redundancy, not resolution alone**: 8–16 sub-pixel
   corners × 3 calibrated views × ~10 Hz temporal fusion, with the 19 cm
   stereo baseline giving geometric conditioning that single-view PnP lacks.
3. **Closed loops beat calibration**: the arm is servoed on TF/FK error, the
   estimate is re-refined from the standoff pose (closer, better pixels), and
   insertion is force-guarded — each loop absorbs the residual of the stage
   before it.
4. **Convention-compatibility is a feature**: same goal composition, same
   phase structure, same thresholds as the Isaac Lab task → every published
   number is directly comparable to the cheat topics for free validation.

## 6. Validation strategy

1. **Spec drift**: `verify_specs.py` — package constants vs `aic_sim.specs` +
   USD (CI-able, no sim needed).
2. **Math unit tests** (no sim): synthetic-projection round-trips — project
   the corner model through known intrinsics/extrinsics, run detector-less
   estimator, require exact recovery; FK vs the USD-authored pose; quaternion
   convention round-trips.
3. **Perception accuracy vs ground truth** (sim): sweep N randomized layouts
   (`aic_sim.randomize` + fixed seeds); compare `/aic/insertion/entrance_pose`
   against `/aic/cheat/entrance_pose`; report mm/deg error distributions per
   axis, per camera count, at 224 vs 448. Gate: p95 lateral < 0.5 mm,
   yaw < 1° at 448.
4. **Closed-loop success rate** (sim): full insertions over N seeds; success =
   internal criteria AND `/aic/cheat/insertion_fraction = 1.0`; log wrench
   peaks, retries, time-to-seat. FK-vs-TF divergence watchdog active.
5. **Robustness probes**: lighting extremes of the DR range, single-camera
   dropout, deliberately biased initial estimate (tests the RETRY path).

## 7. What the live runs changed

The plan above survived contact with the simulator; the details below did
not, and each one is worth more than the plan that preceded it.

1. **The tool transform was wrong, and everything downstream lied.** Reading
   `tcp→tip` from the authored USD xforms (rather than the joint constraint
   frames PhysX actually enforces) put every commanded TCP pose 84 mm off.
   The symptom was not "the arm misses" — it was "the goal orientation looks
   unreachable and the elbow collapses into the stretched singularity",
   which sent me hunting for IK branches, workspace limits and a
   joint-space transit that were all red herrings. **Validate a tool frame
   against the running system before trusting any Cartesian goal built on
   it.** One 30-second measurement against `/tf` would have replaced hours.
2. **Segments must interpolate in TCP space, not tip space.** Rotating about
   the tip orbits the whole 26 cm tool through the workspace boundary;
   rotating about the TCP keeps the wrist nearly still. The upstream planner
   already did this ("shifts each segment endpoint to the TCP frame at plan
   time") — the reason was not obvious until the arm demonstrated it.
3. **The stiff PD drives sag, so the command must lead the measurement.**
   Closing the servo loop on `/joint_states` is necessary but not
   sufficient: an anti-windup clamp of 0.06 rad silently capped the
   achievable correction and parked the tip 20 mm high at every waypoint.
   0.25 rad is still bounded but clears the sag.
4. **Stall detection must be scale-free.** Comparing command to measurement
   flags the (harmless, constant) sag; comparing measured travel to a fixed
   threshold flags the quintic's own rest-to-rest start, which commands only
   ~1.2 mm in its first two seconds. Comparing measured travel to *commanded*
   travel over the same window is quiet in both cases and still catches a
   real jam.
5. **Recovery must return to where it left from.** RETREAT lifts along the
   insertion axis, so a lateral-only "am I at the standoff?" test let each
   retry begin 20 mm higher than the last.

Perception, by contrast, needed no correction: the estimator locked onto the
card during the transit and held `std_xy = 0.14 mm` throughout, and the
standoff arrival landed within 0.8 mm of the predicted pose.

## 8. Implementation plan

1. Package skeleton (`ros2/aic_insertion/`: ament_python, launch, config,
   README) + `specs.py` + `verify_specs.py`.
2. `ur5e_kin.py`: USD-extracted kinematic chain, FK, geometric Jacobian, DLS
   step; unit tests against the authored USD state.
3. `detector.py`: ROI prediction, dark-quad extraction, sub-pixel corners;
   `estimator.py`: ray∩plane, 2-rect fit, gates, fusion, EMA + covariance.
4. `port_perception_node.py` + overlay debug images; validate open-loop
   against cheat topics across randomized layouts (validation step 3).
5. `insertion_node.py`: state machine, quintic segments, servo loop, wrench
   tare/guard, retry logic.
6. `run_sim.py --camera-res`; regenerate nothing (saved stage unaffected at
   default).
7. End-to-end campaign (validation step 4–5), tune thresholds, write results
   into `ros2/aic_insertion/README.md`.
