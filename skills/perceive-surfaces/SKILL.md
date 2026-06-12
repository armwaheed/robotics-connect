---
name: perceive-surfaces
description: >-
  Robot-agnostic real-robot perception: from whatever depth and/or LiDAR a humanoid's descriptor
  declares, detect large flat surfaces (a table, a bed) and project them — and points/pixels on them —
  into the robot's body frame for reaching and placement. Vendor-neutral and LiDAR-first: near-field
  LiDAR is metric-accurate where a stereo/IR depth camera leaves artifacts on textureless bedding, so
  use depth for coarse coverage and LiDAR for the fine hand target. Use this AFTER discover-robot
  (which says what sensors exist) to turn a live scene into actionable body-frame geometry; the sim
  twin is stage-isaac-sensors. One abstraction, driven by the descriptor; the per-sensor algorithms
  live in the robot's capability bindings (no duplication).
metadata:
  tags: [perception, lidar, depth-camera, surface-detection, body-frame, real-to-sim, humanoid, bed-making]
---

# Perceive surfaces (real robot, LiDAR-first)

Turn a live depth/LiDAR scene into **body-frame geometry**: where the flat surface is, and where a
hand should go on it. Robot-agnostic — the *what sensors* comes from the robot
[descriptor](../discover-robot/SKILL.md); the *how to read each one* comes from that robot's
capability bindings. This skill is the orchestration + the principles, not a re-implementation.

## When to use

- A humanoid must **act on a surface** (make a bed, clear a table, place onto a shelf) and needs the
  surface + a grasp/place point **in its own body frame**.
- You have a robot descriptor (from `discover-robot`) and want one vendor-neutral entry point that
  picks the right sensor per query — **LiDAR for the fine target, depth for coarse coverage**.

## The contract

1. **Detect the surface** — height-band + planar-area filter on the point cloud → flat surfaces
   (table/bed) with centroid, extent, and confidence, in body frame (`+x` fwd, `+y` left, `+z` up).
2. **Project a point** — a pixel (depth) or a cloud region (LiDAR) → body-frame XYZ for the hand.
3. **Pick the sensor — LiDAR-first.** Near-field LiDAR is metric to ~±4 cm; a stereo/IR depth camera
   drops invalid pixels (speckle holes) on **textureless bedding and carpet**, so it is coarse-only.
   Use depth for the wide coverage check, LiDAR for the hand target. (Corroborated by the
   cloth-manipulation literature — depth-over-RGB, and LiDAR-over-stereo at folds.)

## Bindings (where the algorithm actually lives)

| Sensor | G1 binding | Vendor-neutral analogue |
|---|---|---|
| LiDAR (Livox MID-360) | [`unitree/g1/lidar_sight`](../../unitree/g1/lidar_sight/SKILL.md) `find_tables()` | any 3D LiDAR cloud → `(N,3)` body-frame points |
| Depth (RealSense D435i) | [`unitree/g1/depth_camera_sight`](../../unitree/g1/depth_camera_sight/SKILL.md) `table_plane_xyz()` / `pixel_to_body_xyz()` | any depth + intrinsics + extrinsic tilt |
| Sim twin | — | [`stage-isaac-sensors`](../stage-isaac-sensors/SKILL.md) `detect_bed()` (same numpy on a RayCaster cloud) |

The **same** detection runs on a simulator's RayCaster cloud and a real LiDAR cloud — that real-to-sim
parity is the whole point: train the detector in sim, run it unchanged on hardware.

## How to use

1. `discover-robot` → descriptor (which depth/LiDAR the robot has, mounts, calibrated tilt, blind spots).
2. For coverage / "is there a surface ahead?", read the **depth** binding (coarse, fast).
3. For the **hand target** on that surface, read the **LiDAR** binding (metric, artifact-free near field).
4. Hand the body-frame XYZ to the reach controller (e.g. the locomotion/reach policy).

> **Do not** trust depth for a fine grasp on textureless bedding — its speckle holes there are exactly
> why this skill is LiDAR-first. If a robot has only a depth camera, surface the coarse-only caveat.
