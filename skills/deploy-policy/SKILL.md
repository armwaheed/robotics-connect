---
name: deploy-policy
description: >-
  Run a trained policy OUTSIDE its RL env — in a control loop or a multi-robot demo, not just the RL
  harness. Use after training a policy with stage-isaac-rl-env. It reconstructs the exact observation
  and action map (obs term order, action = raw*scale + default, joint order) so the exported actor runs
  standalone, and lifts a walk/locomotion ONNX into a torch MLP on the GPU because there is no
  onnxruntime CUDA provider on the DGX Spark (aarch64/GB10). The reach deploy is ambidextrous with no
  observation change. Driven by the robot descriptor; reference impl proven on the Unitree G1.
metadata:
  tags: [deployment, control-loop, onnx, torch, obs-reconstruction, ambidextrous, aarch64, sim-to-real]
---

# Deploy the policy out of the RL env

A policy trained in Isaac Lab is only useful if it runs **outside** the training harness — in a control
loop, a multi-robot demo, or eventually on hardware. That means reproducing the env's observation and
action exactly, by hand. [`scripts/deploy.py`](scripts/deploy.py) does it, lifted from the eye-verified
armwaheed/robots#2 `locomotion.py` and generalized to read the joint set / EE links / action scale from
the [robot descriptor](../discover-robot/SKILL.md).

## 1. Reconstruct the obs/action map exactly

The exported actor is a stateless MLP — feed it the wrong observation and it silently misbehaves. The
`ReachPolicy` rebuilds the **training observation term-for-term**:

```
base_lin_vel(3) base_ang_vel(3) projected_gravity(3) hand_target(7 = base-frame pos+quat)
joint_pos_rel(ALL joints) joint_vel_rel(ALL joints) last_action(len action_joints)
```

and the **action map**: `target = raw * action_scale + default_joint_pos`, applied to the descriptor's
**action joints** (the locked, absent-on-hardware joints are excluded and stay at default — exactly as
in training). The hand target is given in the **base frame** (the policy is yaw-invariant) and **clamped
into the trained command box**, so a receding base can't drive an out-of-distribution command (which the
policy would chase by leaning harder → stepping back → a runaway topple).

```python
import deploy
desc = deploy.load_descriptor(".../descriptors/unitree_g1_29dof.json")
reacher = deploy.ReachPolicy(robot, "cuda", ".../exported/policy.pt", desc, action_scale=0.5)
reacher.set_world_target(headward_corner_xyz)   # aim it; ambidextrous — picks the same-side hand
# per control step (~50 Hz):
reacher.act()
```

**Ambidexterity is free at deploy** — `ReachPolicy` reads the active hand from the (clamped) target's
y-sign, so two robots flanking opposite sides of a bed each lead with their natural same-side hand, with
**no change to the observation**. `active_wrist_link()` tells you which wrist the grasp attaches to.

## 2. Lift the walk ONNX into torch (no onnxruntime GPU on the Spark)

The DGX Spark (aarch64 / GB10, sm_121) has **no onnxruntime CUDA execution provider**, and the project
rule is no CPU inference. `load_onnx_mlp_to_torch` lifts an exported MLP-actor ONNX (a Gemm/ELU stack)
straight into a `torch.nn.Linear` stack on `cuda` — ONNX Gemm `transB=1` stores weight as `(out, in)`,
exactly `torch.nn.Linear` layout, so the weights copy 1:1 (parity ~3e-6).

`VelocityWalker` uses it to run Unitree's velocity-walk policy (480-D obs, 5-step term-major history,
`raw*0.25 + default` action) so the robot **walks to the work site** before the reach policy takes over.
The walk's joint order + default pose come from the descriptor (`sim_asset.joint_order`,
`effectors.default_pose`).

```python
walker = deploy.VelocityWalker(robot, "cuda", ".../g1_velocity_walk.onnx",
                               joint_names=desc["sim_asset"]["joint_order"],
                               default_pos=[...])   # the policy's deploy default
walker.act([vx, vy, wz])   # per control step
```

## The behaviour layer owns the handoff

Walking to a position, deciding to release / retry / ask a peer, and avoiding another robot are
**behaviour-layer** jobs, not terms in the low-level RL policy (training them in would make it
intractable). The deploy pattern is: behaviour picks the target and the walk→reach handoff;
`VelocityWalker` gets the robot there; `ReachPolicy` does the planted, balanced, ambidextrous reach.

## On hardware — the de-risk ladder + low-level takeover

Running the policy in a control loop (above) is not the same as putting it on the **real motors**.
A whole-body policy drives the legs, so deploying it means taking the legs **off the vendor balance
controller** and letting the RL net balance — a first transfer **falls if anything is off, fast
(sub-second)**. Stage it as a ladder; each rung gates the next (full rationale + the stop hierarchy
in [SAFETY.md](../../SAFETY.md)):

| Rung | Runs | Fall risk | Mechanism |
|---|---|---|---|
| **0 offline** | obs+policy, **printed**, no commands | none | read `rt/lowstate`; build the obs; print actions |
| **1 partial** | policy's **arms only**, legs on vendor balance | none | `rt/arm_sdk` weight-blend overlay (`motor_cmd[29].q = weight`) |
| **2 whole-body** | all joints, vendor balance **released** | **HIGH** | `MotionSwitcher.ReleaseMode` + `rt/lowcmd`, robot on a **gantry** |

**Verify the joint order from the ACTUAL env, not just the descriptor.** IsaacLab orders the
articulation DOFs **interleaved (left/right pairs by tree depth)** — e.g. action index 9 is
`left_shoulder_pitch`, sitting *between* the knees (7,8) and ankles (11,12). That is **not** the
Unitree SDK's `0..28` index order. Dump the ground-truth contract by booting the exact training env
and reading `robot.joint_names` / the action term's resolved order + scale + default offset
(`armwaheed/robots#3 rl/dump_deploy_contract.py → deploy_contract.json`), then map sim↔SDK **by joint
name**. Self-check at rung 0: the predicted standing-pose offsets must land on the named joints
(G1 BalanceStand crouch → `left_knee joint_pos_rel ≈ +0.2`, `hip_pitch ≈ −0.17`, arms ≈ 0).

**G1 EDU motor table (verified live):** indexed like the 29-DOF G1 — legs 0–11, `waist_yaw` 12,
L-arm 15–19, R-arm 22–26; the 6 joints the EDU lacks (13,14,20,21,27,28) are **present-but-zero**
and never in the 23-joint action set.

**Rung-2 takeover (the parts that bite):**
- **Release the vendor controller and VERIFY it.** Loop `CheckMode()`/`ReleaseMode()` past transient
  RPC misses (`CheckMode` returns `(code, None)` on a miss); if you can't confirm release, **abort
  without taking over** — never let `rt/lowcmd` fight the vendor controller.
- **Match the sim PD gains** (G1 bed-reach: hip 100/2, knee 150/4, ankle 40/2, waist 200/5, arms 40/10).
  Set `mode_pr=PR`, echo `mode_machine` from `rt/lowstate`, `motor_cmd[i].mode=1`.
- **Motion-blend the handover.** The vendor `BalanceStand` crouch is out-of-distribution for the
  policy (G1 knee ≈ 0.52 vs sim 0.30) — hold the captured pose, then ramp the target from it to the
  policy output over ~2–3 s. Same idea on rung 1 (ramp the `arm_sdk` weight 0→1 holding the live pose).
- **Wrap the loop in [`SafeStop`](../../lib/safe_stop.py)** so it damps on return/exception/SIGINT/
  SIGTERM, and **never `kill -9`** it (a hard kill latches the last high-gain command → runaway; this
  broke a window once — SAFETY.md §0). Abort/end → low-level damp (`kp=0, kd≈3`).

## Generalization

`ReachPolicy` and `VelocityWalker` take the joint set, EE links, and scales from the descriptor, so the
same deploy code runs a 23-DOF or 29-DOF G1, or a new humanoid, once its descriptor and exported policy
exist. The only robot-specific knowledge is in the descriptor — which is the whole point. The hardware
ladder above is likewise robot-agnostic: only the damp/release/gain bindings change per robot, and the
**safe-stop contract ([`lib/safe_stop.py`](../../lib/safe_stop.py)) is mandatory for every robot.**
