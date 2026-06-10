# Onboarding a new humanoid

The skills in this collection are **robot-agnostic**: they are driven by a [robot
descriptor](../schema/robot_descriptor.schema.json), not by hard-coded G1 assumptions. To onboard a
humanoid the agent has never seen, you produce a **new descriptor** and the same staging skills consume
it. This note marks the **generalization seams** — the few places a new robot actually differs — and
works two concrete targets: the Unitree **H2 Plus** and the Boston Dynamics **Atlas**.

## The four seams

Everything robot-specific lives in the descriptor, so onboarding is four steps, each a descriptor field:

1. **Acquire the asset** (`asset_sources`) — get a URDF/USD. Vendor asset server, the vendor SDK, or a
   community description repo. If only a URDF exists, convert to USD with Isaac Lab's URDF importer.
2. **Characterize sensors on the hardware** (`sensors`) — mounts, tilts (floor-plane fit), and the
   robot's own **self-occlusions**. A different head shape blanks different sectors; that is exactly
   the per-robot measurement that makes a sim-trained detector transfer. Reuse the `*_sight` method,
   not the G1's specific numbers.
3. **Characterize effectors** (`effectors`) — actual DOF + joint order from the live SDK, deploy PD
   gains, walking-policy default pose, EE links. Determine which **sim-asset DOFs the robot lacks**.
4. **Stage** (`sim_asset` + the staging skills) — pick a sim asset, set `locked_sim_joints` and the
   hand substitution, then run `stage-isaac-*`.

If those four are filled correctly, the RL env, training, eval, and out-of-env deploy are unchanged.

## Target 1 — Unitree H2 Plus (lowest-friction second robot)

A newer, larger Unitree humanoid. It very likely **reuses the Unitree toolchain** the G1 path already
uses, so most seams are a descriptor diff rather than new infrastructure:

- **Asset** — expect an `h2`/`h2plus` description in `unitree_ros` / the vendor asset tree, and a USD
  on the Isaac asset server (or import the URDF). Same importer path as the G1.
- **Effectors** — a different DOF count, joint set, and gains. Read them from the H2 SDK joint enum the
  same way (`grep` the index map; read the URDF revolute joints). The morphology block captures
  whatever the waist/wrist configuration turns out to be — **don't assume it mirrors the G1's**.
- **Sensors** — likely the same sensor *families* (a head depth camera, a body LiDAR) on a taller frame
  → different mount heights, a different head-cam tilt, and different self-occlusion sectors. Re-run the
  floor-plane tilt fit and the LiDAR occlusion characterization; record the new numbers.
- **Hands** — match finger count to a sim hand as before.
- **Reconciliation** — set `locked_sim_joints` from whatever the H2's actual-vs-asset DOF gap is.

Because the deploy/obs/action conventions carry over, `deploy-policy`'s ONNX→torch lift and obs
reconstruction apply with the H2's joint order substituted in.

## Target 2 — Boston Dynamics Atlas (proves it isn't Unitree-specific)

A different SDK, asset source, and (electric) actuation with a different sensor suite. Atlas is the
stress test for the seams:

- **Asset** — from Boston Dynamics' SDK / description, not `unitree_ros`. The URDF→USD importer path is
  the same; the **joint names and tree differ entirely**, which is why everything downstream reads names
  from the descriptor's `joint_order`/`morphology` rather than hard-coding G1 strings.
- **Effectors** — Atlas's DOF, joint order, and gains come from its own SDK. The descriptor's
  `pd_gains`/`default_pose`/`ee_links` are general enough to hold them. The free-base fix in
  `stage-isaac-freebase` detects and repairs a **baked world-pin on any humanoid USD**, not just the
  G1's `root_joint` — Atlas's root is found and moved to its pelvis-equivalent by the same probe.
- **Sensors** — a different suite (e.g. a perception head with stereo + depth). Add sensor entries with
  their own `type`, `mount_link`, `pose`, and `occlusions`. The `stage-isaac-sensors` skill emits a
  `CameraCfg`/`RayCaster` per entry regardless of robot.
- **Reconciliation** — pick the closest Isaac humanoid asset, lock the DOFs Atlas lacks (or, if the
  asset is lower-DOF, that's a flag to find a better asset), and match the gripper morphology.

The RL env template (`stage-isaac-rl-env`) parameterizes the **action joint set, observation, EE links,
and reward bodies by the descriptor**, so a reach / loco-manipulation task trains on Atlas with no env
rewrite — only a new descriptor.

## What stays the same vs. what changes

| Stays the same (the skills) | Changes (the descriptor) |
|---|---|
| Free-base USD fix (deactivate baked world-pin → move articulation root to the pelvis link) | Which prim is the world-pin / pelvis link |
| Deploy-PD-gains override; stand-up `check_spawn` probe | The gain values; the spawn height |
| Sim sensor configs that reproduce the real blind spots | The mounts, tilts, and occlusion sectors |
| RL env: station-keeping, FALCON grip-slip, SYMDEX ambidexterity, termination-dominant reward | The action joint set, EE links, target workspace |
| Out-of-env deploy: obs reconstruction + ONNX→torch lift | The joint order, obs term order, action scale |

Onboarding a new humanoid is therefore **"write a new descriptor, re-run the skills"** — which is the
acceptance criterion this collection is built to meet.
