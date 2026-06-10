---
name: unitree-g1-sense-lidar
description: >-
  Read the Unitree G1 EDU's crown Livox MID-360 LiDAR on the hardware and characterize its
  self-occlusions (the robot's own face-frame + chin blind spots) and 180° mount-roll for a robot
  descriptor. Use when you need the G1's body-frame point cloud, table/bed detection (find_tables), or
  the LiDAR blind spots that real-to-sim sensing must reproduce in the sim RayCaster. Pure-numpy
  geometry primitives; verified live on a G1 EDU. Feeds the discover-robot descriptor's lidar entry.
metadata:
  tags: [unitree-g1, lidar, livox-mid360, point-cloud, occlusions, blind-spots, table-detection, real-to-sim]
---

# Unitree G1 — crown LiDAR (Livox MID-360)

The crown-mounted Livox MID-360 exposed as a singleton with pure-geometry primitives (cloud access,
horizontal-plane table detection, forward free-space check) plus a scene-map occupancy grid + A\*
planner. Calibration prerequisites, the body-frame transform, the detection algorithm, and on-robot
captures are in **[`README.md`](README.md)** — this skill is the agent entry point.

## When to use

- Get the G1's body-frame **point cloud** (`latest_cloud`) or detect a **table / bed**
  (`find_tables` — height-band + planar area).
- Characterize the LiDAR's **self-occlusions** and **mount roll** for `discover-robot` — the blind spots
  the sim `RayCaster` must reproduce (`stage-isaac-sensors`).

## What it feeds the descriptor

| Descriptor field | From this capability |
|---|---|
| `sensors[].pose.roll_correction_deg` | **180°** — the MID-360 is mounted upside down in the crown; the software roll recovers a clean +x-fwd/+y-left/+z-up body frame. |
| `sensors[].occlusions` | **face-frame** ±40–45° azimuth bars; **chin** below −10° elevation; **dome self-reflection** < 0.15 m radial. The exact blind spots, verified on hardware. |
| `sensors[].pose.xyz_m`, `fov` | 41.6 cm above `torso_link`, 2.3° nose-down; 360° × (−45°, 52°). |
| `sensors[].calibration.reference_media` | `images/scene_side.jpg`, `images/lidar_near_xz.jpg`, `images/lidar_midfar.jpg`. |

> **Don't chase RTX-Livox fidelity in sim.** The real device is characterized here; the sim only needs
> the same blind spots. See `stage-isaac-sensors`.

## Try it (on the robot)

```bash
conda activate unitree_deploy
python lidar_sight.py --frames 20        # live frame rate + cloud shape + a find_tables() pass
python _diag_scene.py --accumulate 5     # z-histogram + floor estimate + find_tables at several bands
python lidar_sight.py --mock             # off-hardware
```

See [`README.md`](README.md) for the calibration prerequisites (remove the face shield, stow the harness
straps) and the full `find_tables` algorithm.
