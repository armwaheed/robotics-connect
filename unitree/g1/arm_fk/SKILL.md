---
name: unitree-g1-arm-fk
description: >-
  Compute Unitree G1 arm forward kinematics (palm/elbow/wrist XYZ) in pure numpy from the bundled URDF,
  in the same body frame the head camera reports surfaces in. Use when you need where the G1's arms are
  (e.g. a palm-over-surface reach trigger), or the G1's kinematic model / joint layout for a robot
  descriptor — including the 23-DOF-vs-29-DOF wrist handling (the EDU's forearms have no wrist
  pitch/yaw). No ROS, no pinocchio; ~1000 Hz on the Jetson, verified on a G1 EDU.
metadata:
  tags: [unitree-g1, forward-kinematics, urdf, arm, palm, dof, 23dof, kinematics]
---

# Unitree G1 — arm forward kinematics

Pure-numpy URDF forward kinematics for the G1 arms: 14-slot `arm_q` → link XYZ in torso or camera body
frame. The full API, frame conventions, the `arm_q` layout, and the reach-trigger use case are in
**[`README.md`](README.md)** — this skill is the agent entry point.

## When to use

- Decide **when the arms have reached a surface** (palm body-frame Z within a few cm of the
  camera-measured surface plane — `arm_fk` + `depth_camera_sight` share a body frame).
- Get the G1's **kinematic model / joint layout** for `discover-robot`.

## The DOF nuance (feeds the descriptor)

The bundled URDF is the **29-DOF** `g1_body29_hand14` model (full wrist chain), but the G1 EDU this runs
against is **23-DOF**: each forearm has **no wrist pitch/yaw** (only `wrist_roll`), and the waist is
yaw-only. In the 14-slot `arm_q`, the two wrist slots per arm (indices 5/6 and 12/13) are **phantom** —
always 0 on the 23-DOF robot, and the URDF's wrist-pitch/yaw joints fold to identity at their fixed
offsets, which is exactly the correct palm geometry.

This is the per-robot morphology `discover-robot` records in the descriptor's `effectors.morphology`
(`absent_joints`: `*_wrist_pitch_joint`, `*_wrist_yaw_joint`, `waist_roll/pitch_joint`) and reconciles
against the 29-DOF sim asset (`sim_asset.sim_real_reconciliation.locked_sim_joints`). The palm is a fixed
offset from the forearm, so a correct palm XYZ does not depend on the missing wrists.

> The URDF models Unitree's stock rubber hand, **not** the Brainco V2 hands on the EDU. Palm position is
> correct regardless (fixed offset from the wrist); per-finger FK would need a Brainco sub-chain — see
> [`unitree-g1-hands`](../brainco_touch/SKILL.md).

## Try it (no hardware needed)

```bash
python arm_fk.py          # prints REST/EXTEND palms, then SELFTEST_OK
python test_arm_fk.py     # 9 regression tests
```

See [`README.md`](README.md) for the API, the body/torso frame conventions, and the reach-check example.
