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

## Generalization

`ReachPolicy` and `VelocityWalker` take the joint set, EE links, and scales from the descriptor, so the
same deploy code runs a 23-DOF or 29-DOF G1, or a new humanoid, once its descriptor and exported policy
exist. The only robot-specific knowledge is in the descriptor — which is the whole point.
