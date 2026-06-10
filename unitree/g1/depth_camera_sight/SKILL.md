---
name: unitree-g1-sense-depth
description: >-
  Read the Unitree G1 EDU's head Intel RealSense D435i depth + RGB on the hardware, and characterize its
  floor-plane-calibrated downward tilt for a robot descriptor. Use when you need the G1's head-camera
  frames, body-frame table/surface geometry, or the head-cam down-tilt angle that real-to-sim sensing
  needs (the head camera is angled down, sees the floor + near surface, NOT the room). Import-and-call
  Python; verified live on a G1 EDU. Feeds the discover-robot descriptor's camera sensor entry.
metadata:
  tags: [unitree-g1, depth-camera, realsense, rgb, head-tilt, sensor, real-to-sim, calibration]
---

# Unitree G1 — head depth + RGB camera

The head Intel RealSense D435i exposed as import-and-call Python, plus the RGB stream over the Unitree
video hub. Full install, try-it commands, coordinate conventions, and on-robot captures are in
**[`README.md`](README.md)** — this skill is the agent entry point; don't duplicate it, read it.

## When to use

- Get the G1 head camera's **depth + RGB** frames, or body-frame **table/surface geometry**
  (`pixel_to_table_xy`, `table_plane_z`) — in the same body frame `arm_fk` uses.
- Characterize the head camera's **downward tilt** for `discover-robot` (the calibrated value sets the
  sim `CameraCfg` pitch — see `stage-isaac-sensors`).

## The hard constraint (real-to-sim)

The head D435i is mounted **pointing down**. It sees the floor, the near surface, and objects on it —
**not** standing humans at conversational range or the broader room. Every consumer must be designed
around that geometry, and the sim camera must share it.

## What it feeds the descriptor

| Descriptor field | From this capability |
|---|---|
| `sensors[].pose.tilt_deg` | `CAMERA_TILT_DEG_DEFAULT` — floor-plane SVD fit (`_calibrate_tilt.py`). **51.29°** on the dev EDU; **re-measure per robot**. |
| `sensors[].mount_link`, `pose.xyz_m` | Head module on `torso_link`. |
| `sensors[].calibration.reference_media` | `images/rgb_sample.jpg`, `images/depth_sample.jpg` — the "what a calibrated frame looks like" reference. |

## Try it (on the robot)

```bash
source setup_env.sh                                    # LD_LIBRARY_PATH, PYTHONPATH, LD_PRELOAD
export CYCLONEDDS_URI=file:///home/unitree/cyclonedds.xml
python _diag_camera_test.py                            # one-shot "is the camera alive"
python _calibrate_tilt.py --camera-height-m 1.242 --bottle-forward-m 0.9144   # re-fit the tilt
```

See [`README.md`](README.md) for the full guide, the RGB-via-DDS path, and the gotchas.
