# Robotics Connect — Agent Skills

This is the skill catalog for [`robotics-connect`](../README.md). The skills here let an
AI agent (Claude Code) take a humanoid **from plugged-in on the bench → to a sim-to-real-valid
Isaac Sim RL training job on a DGX Spark**, and do it for a robot it has **never seen before**.

Everything here was learned doing [armwaheed/robots#2](https://github.com/armwaheed/robots/issues/2)
(two Unitree G1s make a bed in Isaac Sim under whole-body loco-manipulation RL, no kinematic
cheats). The skills are the reusable distillation of that work.

## The real-to-sim thesis (why this exists)

Isaac Sim does **not** fully expose a real robot's sensor + effector envelope. The sensors are
tilted, range-limited, and occluded by the robot's own body in ways the sim won't tell you; the
real degrees of freedom, gains, and hand morphology differ from whatever asset you downloaded.
`robotics-connect` **characterizes those on the hardware**, and that characterization is
**calibration data an agent builds the sim from** — so a sim-trained policy/detector transfers,
and the RL training cycles are spent on a sim that already matches the robot. robotics-connect is
the **real-to-sim** half that feeds sim-to-real.

The artifact at the center of that loop is the **robot descriptor** (`discover-robot`): a single
machine-readable card describing the *real* robot — its actual DOF and joint order, its PD gains,
its sensor mounts/tilts/blind-spots, its hand morphology, and **how it reconciles with the sim
asset** (which sim DOFs the real robot lacks). Every Isaac-staging skill consumes that descriptor.

## Layout — two kinds of skill

| Kind | Lives in | Why |
|---|---|---|
| **Robot-agnostic** — discovery, Isaac staging, host setup | `skills/` (flat, this dir) | One implementation, driven by a robot descriptor; works for any humanoid. |
| **Robot-scoped** — per-capability control/perception | `<manufacturer>/<product>/<capability>/SKILL.md` | Co-located with the verified-on-hardware code, keeping the manufacturer/product layout. |

Robot-scoped skills stay next to the code they wrap (e.g. `unitree/g1/lidar_sight/SKILL.md`) so the
verified stack and its agent entry point live together. Both trees are registered as skill
directories in [`.claude-plugin/plugin.json`](../.claude-plugin/plugin.json), so an agent discovers
all of them uniformly.

## The skills

### Robot-agnostic — discovery + real-to-sim staging

| Skill | When to use it |
|---|---|
| [`discover-robot`](discover-robot/SKILL.md) | **The keystone.** Inventory a connected robot's sensors + effectors and emit its machine-readable **robot descriptor** — the real-to-sim card every staging skill below consumes. Run this first for any robot, known or new. |
| [`stage-isaac-sensors`](stage-isaac-sensors/SKILL.md) | Turn the descriptor's sensor characterization into Isaac Sim sensor configs that **share the real sensor's envelope + blind spots** (head-cam down-tilt, LiDAR self-occlusions), so a sim-trained detector transfers. |
| [`stage-isaac-freebase`](stage-isaac-freebase/SKILL.md) | Get a manufacturer USD/URDF into Isaac Lab as a **free-base** articulation with **deploy PD gains** and no kinematic cheats — including the baked-world-pin fix and **locking the sim DOFs the real robot lacks**. |
| [`stage-isaac-rl-env`](stage-isaac-rl-env/SKILL.md) | Stand up a runnable manager-based Isaac Lab RL **reach / loco-manipulation** env from the descriptor: station-keeping, FALCON-style grip-slip load, SYMDEX ambidexterity, the training/eval gotchas baked in. |
| [`deploy-policy`](deploy-policy/SKILL.md) | Run a trained policy **outside** its RL env (control loop / multi-robot demo): reconstruct the exact obs/action map, and lift a walk ONNX into a torch MLP (no onnxruntime GPU on the Spark). |
| [`setup-dgx-spark`](setup-dgx-spark/SKILL.md) | Bring up Isaac Sim + Isaac Lab on a **DGX Spark (GB10, aarch64)** — the non-obvious gotchas (source build, `LD_PRELOAD`, rsl_rl shim, Fabric render, headless eval video). |

### Robot-scoped — Unitree G1 EDU capabilities (verified on hardware)

| Skill | Capability |
|---|---|
| [`unitree/g1/depth_camera_sight`](../unitree/g1/depth_camera_sight/SKILL.md) | Head Intel RealSense depth + RGB; floor-plane-calibrated **51.29° down-tilt**; body-frame geometry. **Depth needs `pyrealsense2` built from source (`install_pyrealsense2.sh`) + `setup_env.sh` sourced** — not a pip install (GLIBC). RGB is dependency-free (DDS). |
| [`unitree/g1/lidar_sight`](../unitree/g1/lidar_sight/SKILL.md) | Crown Livox MID-360; table/bed detection; **face-frame / chin self-occlusions** characterized on hardware. |
| [`unitree/g1/arm_fk`](../unitree/g1/arm_fk/SKILL.md) | Pure-numpy URDF forward kinematics; palm XYZ in the camera body frame; **23-DOF-vs-29-DOF** wrist handling. |
| [`unitree/g1/brainco_touch`](../unitree/g1/brainco_touch/SKILL.md) | Brainco 5-finger hands — digits, touch, proximity (the real hand; the sim uses Inspire). |
| [`unitree/g1/vision_sidecar`](../unitree/g1/vision_sidecar/SKILL.md) | Containerized GPU (DINOv2) inference sidecar over local RPC. |
| [`unitree/g1/install`](../unitree/g1/install/SKILL.md) | On-robot deploy / uninstall / offline-bundle of the stack. |
| [`unitree/g1/connect`](../unitree/g1/connect/SKILL.md) | Route a control host to the robot's subnet (`configure_*.sh` + CycloneDDS). |

## A typical end-to-end flow

```
discover-robot                 →  robot descriptor (sensors + effectors, real DOF, sim reconciliation)
  ├─ unitree/g1/depth_camera_sight, lidar_sight, arm_fk, brainco_touch   (characterize on hardware)
setup-dgx-spark                →  Isaac Sim 5.1 + Isaac Lab 2.3 live on the Spark
stage-isaac-sensors            →  sim cameras/ray-casters with the real blind spots
stage-isaac-freebase           →  free-base USD + deploy gains + locked-DOF reconciliation
stage-isaac-rl-env             →  runnable Isaac Lab RL job
  → train → eval (verify by eye) →
deploy-policy                  →  policy in a control loop / multi-robot demo
```

To onboard a **new** humanoid (Unitree H2 Plus, Boston Dynamics Atlas, …), re-run `discover-robot`
to produce a new descriptor and the same staging skills consume it. See
[`discover-robot/references/onboarding-new-humanoid.md`](discover-robot/references/onboarding-new-humanoid.md).

## Validation media is a first-class asset

For an agent discovering a robot or applying a skill to a novel one, the captured media are the
**reference standards** — what a calibrated sensor's envelope looks like, what RL convergence looks
like, and what a good policy looks like **by eye** (this project's hard rule: *verify by what a
human sees, never by reward telemetry alone*). The skills carry that media as assets under
[`assets/media/`](../assets/media/) and in each robot-scoped capability's `images/`.
