---
name: stage-isaac-freebase
description: >-
  Get a manufacturer humanoid USD into Isaac Lab as a free-base articulation with deploy PD gains and no
  kinematic cheats — and reconcile the sim asset's DOF with the real robot's. Use when staging the robot
  for an Isaac Sim RL job. Fixes the common baked-world-pin failure (many humanoid USDs pin the pelvis to
  the world via a root_joint that fix_root_link only disables, then "Failed to create articulation"),
  sets the robot's deploy gains (stock manipulation gains collapse a balance policy), takes the neutral
  pose from the walking-policy default, and LOCKS the sim joints the real robot lacks (e.g. a 23-DOF G1
  EDU on a 29-DOF asset) so a trained policy is transfer-valid. Driven by the discover-robot descriptor.
metadata:
  tags: [isaac-lab, free-base, articulation, usd, pd-gains, dof-reconciliation, sim-to-real, humanoid]
---

# Stage a free-base humanoid — no cheats, DOF reconciled

A whole-body balance policy is only physically valid if the simulated robot is **free-base** (not
bolted to the world), runs **deploy gains**, and has the **same effective DOF as the hardware**. This
skill produces exactly that from the [robot descriptor](../discover-robot/SKILL.md).

Scripts (lifted + generalized from the eye-verified armwaheed/robots#2 `rl/`):

| Script | Does |
|---|---|
| [`scripts/make_mobile_usd.py`](scripts/make_mobile_usd.py) | One-time: author a free-base override USD (the world-pin fix). |
| [`scripts/robot_cfg.py`](scripts/robot_cfg.py) | Build the `ArticulationCfg` from the descriptor: deploy gains, neutral pose, **locked-DOF reconciliation**, and `action_joints()`. |
| [`scripts/check_spawn.py`](scripts/check_spawn.py) | Validate the spawn stands under gravity before training. |

## 1. The free-base fix (an open NVIDIA problem, solved)

Many humanoid USDs ship fixed-base + gravity-off for tabletop manipulation and bake the
`ArticulationRootAPI` onto a `root_joint` `PhysicsFixedJoint` that pins the pelvis to the world.
Setting `fix_root_link=False` only **disables** that joint; Isaac then can't resolve the articulation
root → `Failed to create articulation`. (For the G1 Inspire-hand this is NVIDIA forum **370590**,
unanswered.)

`make_mobile_usd.py` writes a ~1 KB override layer that **deactivates the baked joint** and **moves the
articulation root onto the base link**. It auto-detects the world-pin (a fixed joint with no `body0`),
so it generalizes — `--root-link pelvis` for the G1, the robot's base link for anything else:

```bash
./isaaclab.sh -p scripts/make_mobile_usd.py \
    --src "${ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/G1/g1_29dof_inspire_hand.usd" \
    --out ./assets/g1_inspire_mobile.usd --root-link pelvis
```

## 2. Deploy PD gains (the stock gains collapse the policy)

The stock manipulation preset is far too stiff for balance (the G1 Inspire preset uses waist kp 5000 /
arms kp 3000). `robot_cfg.make_robot_cfg` replaces them with the descriptor's `effectors.pd_gains` (the
*deploy* gains). The neutral pose comes from `effectors.default_pose` — the robot's **walking-policy
default** — so the reach policy's neutral matches the deploy stance and the walk→reach handoff lands
in-distribution.

## 3. DOF reconciliation — the real-to-sim crux for a mismatched asset

The sim asset often has **more DOF than the real robot**. Training a policy that balances/reaches using
joints the hardware doesn't have produces a sim result that **never transfers**. The descriptor's
`sim_asset.sim_real_reconciliation.locked_sim_joints` names those joints, and `robot_cfg`:

- puts them in a **stiff "locked" actuator group** that holds them at their default, and
- **excludes** them from the policy's action set — `action_joints(descriptor)` returns
  `sim_asset.joint_order − locked_sim_joints`.

```python
import robot_cfg as rc
desc = rc.load_descriptor("../discover-robot/descriptors/unitree_g1_edu.json")
rc.action_joints(desc)   # 23 joints — the 6 absent-on-hardware joints are excluded
rc.locked_joints(desc)   # ['waist_roll_joint','waist_pitch_joint','left_wrist_pitch_joint', ...]

cfg = rc.make_g1_cfg(desc, mobile_usd="./assets/g1_inspire_mobile.usd")
```

For the **23-DOF G1 EDU** this locks 6 joints, so the policy trains a 23-DOF action it can run on the
hardware. For a **29-DOF G1** the locked list is empty — the policy commands all 29, and the issue#2
reach policy is directly transfer-valid. Same builder, descriptor decides.

> **Honest caveat carried in the descriptor.** The issue#2 bed-reach policy was trained on the *full*
> 29-DOF asset (it leans using waist pitch). That is a valid sim result but **not** directly
> transfer-valid to a 23-DOF robot — retrain with `locked_sim_joints` applied for that hardware.

## 4. Validate before you train

```bash
./isaaclab.sh -p scripts/check_spawn.py \
    --descriptor ../discover-robot/descriptors/unitree_g1_edu.json \
    --mobile-usd ./assets/g1_inspire_mobile.usd
```

`check_spawn` confirms the articulation is free-base, every action joint + the EE/foot/pelvis bodies
resolve, and the robot **stands for 3 s under gravity** on the deploy gains. If it can't stand at zero
action, the gains or spawn height are wrong — fix that before spending a training run.

## Hand-off

`action_joints(descriptor)`, the EE links, and the built `ArticulationCfg` feed straight into
`stage-isaac-rl-env`, which parameterizes the action term and reward bodies by them.
