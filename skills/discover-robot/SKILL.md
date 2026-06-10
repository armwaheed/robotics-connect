---
name: discover-robot
description: >-
  Onboard a humanoid robot — connected on the bench or a model you've never seen — by characterizing
  its REAL sensors and effectors on the hardware and emitting a machine-readable robot descriptor (the
  real-to-sim calibration card). Use this FIRST for any robot before staging an Isaac Sim RL job. It
  reads the robot's actual degrees of freedom and joint order from the live SDK (not the asset's), its
  deploy PD gains and neutral pose, its sensor mounts/tilts/blind-spots, and its hand morphology, then
  reconciles them against the Isaac Sim asset (which sim DOFs the real robot lacks). Works for any
  humanoid; worked examples for the full 29-DOF Unitree G1 and the reduced 23-DOF G1 EDU are included.
metadata:
  tags: [discovery, onboarding, descriptor, real-to-sim, humanoid, dof, sensors, calibration]
---

# Discover Robot — emit the real-to-sim descriptor

This is the **keystone** skill. Every Isaac-staging skill (`stage-isaac-sensors`,
`stage-isaac-freebase`, `stage-isaac-rl-env`, `deploy-policy`) consumes the **robot descriptor** this
skill produces. The descriptor is the real-to-sim calibration card: a single machine-readable file
describing the *real* robot so the simulator can be built to match it.

> **No robot is the paradigm.** A robot may be a full **29-DOF** G1 (the common research config), a
> reduced **23-DOF** G1 EDU, a Unitree H2 Plus, a Boston Dynamics Atlas, or anything else. The whole
> point of this skill is to **discover what the robot actually is** and write it down — never to
> assume. The descriptor schema is robot-agnostic; the staging skills are driven entirely by it.

- **Schema:** [`schema/robot_descriptor.schema.json`](schema/robot_descriptor.schema.json)
- **Examples:** [`descriptors/unitree_g1_29dof.json`](descriptors/unitree_g1_29dof.json) (full DOF,
  the clean sim==real case) · [`descriptors/unitree_g1_edu.json`](descriptors/unitree_g1_edu.json)
  (the reduced 23-DOF variant that needs sim joints locked — validated on the live robot)
- **Onboarding a new humanoid:** [`references/onboarding-new-humanoid.md`](references/onboarding-new-humanoid.md)

## The procedure

### 1. Identify the robot and acquire its asset
Determine `manufacturer`, `product`, and the as-built `variant`. Acquire the kinematic/visual asset
(URDF from the vendor SDK / `unitree_ros` / `g1_description`; USD for Isaac Sim). Record both under
`asset_sources`.

### 2. Characterize effectors on the hardware — read the ACTUAL DOF
**Do not trust the asset's DOF count.** Two units of the same product can differ; an asset is often a
higher-DOF superset of the robot in front of you. Read the truth from the live robot:

- **Joint set + order** — from the robot's deploy/SDK joint enum and its URDF. On a Unitree G1:
  ```bash
  # Canonical motor index map (note the comments marking joints INVALID on reduced variants):
  grep -E "= [0-9]+" unitree_sdk2_python/example/g1/low_level/g1_low_level_example.py
  # The actuated joints of the as-built robot (revolute joints in its own URDF):
  grep -E "<joint" g1_description/g1_<variant>.urdf | grep -i revolute
  ```
  Fill `effectors.dof`, `effectors.joint_order`, and the per-segment `effectors.morphology`
  (`present_joints` / `absent_joints`). The `absent_joints` are the seam the sim is reconciled against.
- **Deploy PD gains** (`effectors.pd_gains`) — extract from the robot's deploy config. The stock
  *manipulation* gains are typically far too stiff for a whole-body balance policy and must be
  replaced with the *deploy* gains (see `stage-isaac-freebase`).
- **Neutral pose** (`effectors.default_pose`) — take it from the robot's own **walking-policy default**
  so a reach policy's neutral matches the deploy stance (clean walk→reach handoff).
- **End-effector links** (`effectors.ee_links`) — on a reduced-DOF arm the distal *actuated* link may
  differ from the canonical wrist link; note it.

### 3. Characterize sensors on the hardware
Run the robot-scoped `*_sight` capability skills (for the G1: `unitree/g1/depth_camera_sight`,
`unitree/g1/lidar_sight`) and record, per sensor, the `mount_link`, `pose` (including any **down-tilt**
calibrated by a floor-plane fit and any **roll correction** for a rotated mount), `fov`, and — the
point of real-to-sim sensing — the **`occlusions`** the robot's own body imposes (azimuth bands behind
a face frame, an elevation floor under a chin, self-reflection inside a dome). Point `calibration.method`
and `calibration.reference_media` at the on-hardware captures that are the "what a calibrated envelope
looks like" reference. **These blind spots are robot-specific — measure them per robot.**

### 4. Characterize the hands
Record `hands.model`, `fingers`, `dof`, `control`, `tactile`. Finger COUNT is what matters for picking
a sim hand (see step 5) — match morphology, not brand.

### 5. Reconcile with the sim asset, then emit
Fill `sim_asset`: the Isaac USD, its DOF, its hand, and the **`sim_real_reconciliation`**:
- `locked_sim_joints` — sim joints present in the asset but **absent on the real robot**. Hold these at
  default and exclude them from the policy's action set, so the trained policy only commands DOF the
  hardware has. **For a sim==real robot (e.g. a full 29-DOF G1 staged against the 29-DOF asset) this
  list is empty** — nothing to lock.
- `hand_substitution` — the sim hand chosen for the real hand, by finger count.

Validate the file against the schema before handing it off:
```bash
python -c "import json,jsonschema,sys; \
  s=json.load(open('skills/discover-robot/schema/robot_descriptor.schema.json')); \
  d=json.load(open('<your_descriptor>.json')); jsonschema.validate(d,s); print('descriptor OK')"
```

### 6. Hand off
Pass the descriptor to `setup-dgx-spark` (host) then `stage-isaac-sensors` → `stage-isaac-freebase` →
`stage-isaac-rl-env` → train → eval (verify by eye) → `deploy-policy`.

## Worked example — the validated G1 EDU (23-DOF)

[`descriptors/unitree_g1_edu.json`](descriptors/unitree_g1_edu.json) was produced by running this
procedure against a live G1 EDU. It is a *reduced* variant — a useful hard case because the sim asset
is a higher-DOF superset:

- **23 actuated joints** = 12 legs + `waist_yaw` + 5/arm (shoulder p/r/y, elbow, **wrist_roll only**).
  The waist is yaw-only; the arms have no wrist pitch/yaw.
- The Unitree SDK enum marks exactly the missing six — `WaistRoll`, `WaistPitch`, and L/R
  `WristPitch`/`WristYaw` — as **"INVALID for g1 23dof"**.
- Those six become `sim_asset.sim_real_reconciliation.locked_sim_joints`, so a policy trained on the
  29-DOF Inspire asset can be made transfer-valid by locking them.
- Hands: real **Brainco** 5-finger ↔ sim **Inspire** 5-finger (match finger count; the Isaac default
  Dex3 3-finger was rejected).

Contrast [`descriptors/unitree_g1_29dof.json`](descriptors/unitree_g1_29dof.json): a full-DOF G1 where
the sim asset matches the robot, `locked_sim_joints` is empty, and the issue#2 reach policy is directly
transfer-valid. Same skill, same schema — the descriptor simply records what each robot is.
