# arm_fk — G1 forward kinematics

Pure-numpy URDF-based forward kinematics for the Unitree G1 arms.

All source lives in `robotics-connect/arm_fk/`.

> **If you're a coding agent looking for a kinematic model of the G1,
> this is it.** No ROS, no pinocchio, no urchin, no mesh loader. A
> single `pip list` on the robot's `unitree_deploy` env shows numpy
> and that's all this package needs. Import it like any other module
> under `unitree/`, call `G1ArmFK()`, and you have a 14-DOF arm chain
> rooted at `torso_link` with palm XYZ in the same body frame that
> `depth_camera_sight.pixel_to_body_xyz` reports the table plane in.

## The 60-second tour

```python
import sys; sys.path.insert(0, "robotics-connect/arm_fk")
from arm_fk import G1ArmFK
import numpy as np

fk = G1ArmFK()                                  # loads the bundled URDF once
arm_q = np.zeros(14, dtype=np.float32)          # 14-slot arm_q layout

torso  = fk.forward(arm_q)                      # dict of link XYZ in torso frame
body   = fk.forward_body_frame(arm_q)           # same, origin = d435 camera mount
palms  = fk.palm_xyz(arm_q)                     # just {"L_palm","R_palm"}

stats = fk.benchmark(n=1000)
print(f"{stats['per_call_us']:.1f} us/call, {stats['hz']:.0f} Hz")
```

Run the self-test any time to confirm the URDF is loading cleanly:

```bash
python robotics-connect/arm_fk/arm_fk.py         # prints REST_POSE and EXTEND_POSE palms, then SELFTEST_OK
python robotics-connect/arm_fk/test_arm_fk.py    # 9 regression tests
```

Both scripts exit 0 on success and print `SELFTEST_OK` / `ALL_TESTS_PASSED`.

## Why it exists

A common need is to decide when the arms have reached down to a surface.
A hand-tuned joint-space threshold:

```python
if shp_l >= -0.510 and shp_r >= -0.510 and ...:  # stopgap
    reached = True
```

works on one robot, one stance, one table height. Every unit with
slightly different joint encoder offsets, base-pose drift, or table
height would need hand re-tuning. The principled version is:

> fire when either palm's body-frame Z is within 3 cm of the
> camera-measured table plane.

which requires forward kinematics from the 14-DOF arm joints to palm XYZ
in the camera's body frame. That is this package: import `G1ArmFK` as a
soft dependency and fall back to the shoulder-pitch threshold only when
`arm_fk` or the depth camera are unavailable.

## What's in the box

```
arm_fk/
├── __init__.py                 — public API (G1ArmFK, friendly link name constants)
├── arm_fk.py                   — parser + FK + selftest (the main file)
├── test_arm_fk.py              — 9 regression tests, runs directly (no pytest)
├── README.md                   — this file
└── urdf/
    └── g1_body29_hand14.urdf   — Unitree's stock G1 URDF, bundled (51 KB)
```

The bundled URDF is the 29-DOF `g1_body29_hand14` model from Unitree's
own `unifolm-world-model-action` asset tree. We use the 29-DOF variant
because it contains the full wrist chain (`wrist_roll`, `wrist_pitch`,
`wrist_yaw`) that the forearm-to-palm offset needs, even though the
G1 EDU we run against is 23-DOF. On the 23-DOF hardware, the two
phantom wrist slots in the 14-DOF `arm_q` array (indices 5/6 and 12/13)
are always 0 and the wrist-pitch/wrist-yaw joints in the URDF chain
fold to identity rotations at their fixed offsets — which is exactly
the correct geometric palm position.

## Frame conventions

There are two output frames:

| Frame | Origin | Axes | Use case |
|---|---|---|---|
| **Torso frame** | `torso_link` | +X forward, +Y left, +Z up | Cross-module chain composition, URDF-native queries |
| **Body frame** | `d435_link` (camera mount) | +X forward, +Y left, +Z up | **Compare directly against `depth_camera_sight.pixel_to_body_xyz`** |

The body frame is the one you almost always want. It matches
`depth_camera_sight` exactly, so `body_frame_palm_z - table_plane_z` is
a direct geometric gap in metres. Using this frame means the FK trigger
never needs to know where the torso is relative to anything — the
camera is the origin and the camera already sees the table plane.

## What it does NOT do

- **Fingertip FK on Brainco V2 hands.** The bundled URDF models
  Unitree's stock 7-joint rubber hand. The palm position is correct
  regardless of which hand is bolted on (the palm is a fixed offset
  from the wrist), but any per-finger link position will be geometrically
  wrong on the Brainco V2. For palm-level reach handoff,
  this doesn't matter. For any future feature that needs fingertip
  XYZ (visual fingertip servoing, finger-to-object distance), the fix
  is a Brainco-specific sub-chain override.
- **Self- or world-collision detection.** The parsed `_Joint` tree is
  there, but the collision side (`trimesh` / `FCL` queries against the
  meshes referenced in the URDF) is not implemented. Listed as future
  work.
- **Visualization / rendering.** No mesh loading, no window, no browser.
- **Dynamics.** Kinematic only. No mass, no inertia, no gravity, no
  contact. For physics-grade queries, consider an external `unitree_mujoco`
  XML model as an alternative backend.

## 14-DOF arm_q layout

```
 0  left_shoulder_pitch    IDX_L_SHPITCH
 1  left_shoulder_roll     IDX_L_SHROLL
 2  left_shoulder_yaw      IDX_L_SHYAW
 3  left_elbow             IDX_L_ELBOW
 4  left_wrist_roll        IDX_L_FOREARM      (a.k.a. "forearm roll")
 5  phantom                (left_wrist_pitch on 29-DOF; 0 on 23-DOF EDU)
 6  phantom                (left_wrist_yaw   on 29-DOF; 0 on 23-DOF EDU)
 7  right_shoulder_pitch   IDX_R_SHPITCH
 8  right_shoulder_roll    IDX_R_SHROLL
 9  right_shoulder_yaw     IDX_R_SHYAW
10  right_elbow            IDX_R_ELBOW
11  right_wrist_roll       IDX_R_FOREARM
12  phantom                (right_wrist_pitch)
13  phantom                (right_wrist_yaw)
```

`arm_fk.ARM_JOINT_NAMES_14` in `arm_fk.py` is the one-line constant that
defines this index-to-joint mapping. Keep any caller's joint ordering in
lockstep with it.

## Example: palm-over-table reach check

A control loop can use `arm_fk` as the primary reach trigger whenever
both `arm_fk` and a depth camera are available:

```python
palms_body = arm_fk.palm_xyz(arm_q_now)
table_z    = camera.table_plane_z() or DEFAULT_TABLE_Z_BODY
min_gap    = min(palms_body["L_palm"][2], palms_body["R_palm"][2]) - table_z
if min_gap < REACH_PALM_OVER_TABLE_M:     # 3 cm
    reached = True
    break
```

Print a 1 Hz `[fk]` diagnostic line with `L_palm.z`, `R_palm.z`,
`table_z`, and `gap_min` during a live run so operators can see exactly
what the trigger is seeing. Suppress the shoulder-pitch fallback when
camera-FK is enabled so the two triggers never race.

## Benchmark

| Target | per-call | Hz |
|---|---|---|
| Workstation (numpy 1.24, Ryzen) | ~225 µs | ~4400 |
| G1 Jetson (`unitree_deploy` env) | ~1001 µs | ~1000 |

Both are >100× faster than the 10 Hz REACH control loop, so FK is
noise relative to DINOv2 / retrieval / DDS. No need to cache.

## Known calibration delta

The URDF's `d435_joint` origin has pitch 0.8307767 rad (47.6°), but
`depth_camera_sight`'s floor-plane SVD calibration measures 51.29° on
our physical G1 EDU. The 3.7° delta is most likely live `waist_pitch`
absorbed into the chain at standing balance.

**For this package it does not matter** — we only use the
translation component of `camera_offset_torso`, not the rotation, and
the body frame is defined by the origin of `d435_link`, not its
orientation. If a future feature needs the camera's optical orientation
in body frame (hand-eye, visual servoing), that's where to pick which
number to believe and whether to read `waist_pitch` live.
