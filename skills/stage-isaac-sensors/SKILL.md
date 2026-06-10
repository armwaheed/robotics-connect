---
name: stage-isaac-sensors
description: >-
  Turn a robot descriptor's hardware-characterized sensors into Isaac Sim sensor configs that SHARE
  the real sensor's envelope and blind spots, so a sim-trained detector transfers to the robot. Use
  when staging the perception half of an Isaac Sim RL job: it emits a head CameraCfg at the real
  floor-plane-calibrated down-tilt, and a RayCaster LiDAR that reproduces the robot's own
  self-occlusions (face-frame azimuth bands, chin elevation floor, dome self-reflection). Do NOT chase
  RTX-Livox fidelity — the real device is characterized on hardware; the sim only needs the same blind
  spots. Consumes the descriptor from discover-robot; reference impl proven on the Unitree G1.
metadata:
  tags: [isaac-sim, sensors, perception, real-to-sim, lidar, camera, raycaster, blind-spots]
---

# Stage Isaac sensors — calibrated to the real ones

Isaac Sim does not expose a real robot's sensor envelope. The sensors are tilted, range-limited, and
**occluded by the robot's own body** in ways the sim won't tell you. This skill builds the simulated
sensors from the **robot descriptor** (produced by `discover-robot`, where those characteristics were
measured on the hardware by the `*_sight` capability skills), so a detector trained in sim works on the
robot.

**Reference implementation:** [`scripts/perception.py`](scripts/perception.py) — lifted from the
eye-verified armwaheed/robots#2 implementation and generalized to read sensor poses + occlusions from
the descriptor (the G1 EDU values remain the defaults).

## What it emits

| Real sensor (from the descriptor) | Sim config it builds | Shared characteristic |
|---|---|---|
| Head depth/RGB camera, calibrated **down-tilt** | `CameraCfg` at that pitch on the mount link | Frames the floor + near surface, **not the room** (so the sim sees what the robot sees). |
| Body LiDAR (e.g. Livox MID-360) | `RayCaster` at the real FOV | Same vertical/horizontal FOV; nose-down + roll-corrected mount. |
| The robot's **self-occlusions** | `occlude()` applied to the cast cloud | Face-frame azimuth bands, chin elevation floor, dome self-reflection — **the exact blind spots**. |

```python
# Driven entirely by the descriptor — no robot-specific code:
import json, perception
desc = json.load(open("skills/discover-robot/descriptors/unitree_g1_edu.json"))
cfgs = perception.build_sensor_cfgs(desc, robot_prim="{ENV_REGEX_NS}/Robot",
                                    mesh_prim_paths=["/World/ground", "{ENV_REGEX_NS}/Bed"])
# cfgs == {"head_depth": CameraCfg(...51.29° down...), "crown_lidar": RayCaster(...)}

# At read time, blank the body-blocked sectors so the sim cloud matches the hardware:
occ = perception.occlusions_for(desc, "crown_lidar")
visible = perception.occlude(cloud_body_frame, occ)
```

## Why not full RTX-Livox fidelity

Chasing a physically exact Livox replay in RTX is computationally infeasible **and a misdirection**.
The real device is already characterized on the hardware — the sim only needs to reproduce the same
**blind spots** for a detector to transfer. A geometric `RayCaster` plus the descriptor's `occlude()`
is the right fidelity. (This was a real trap on armwaheed/robots#2; don't re-enter it.)

## Bed / large-flat-surface detection

`detect_bed()` mirrors `lidar_sight.find_tables` widened from a table to a bed: occlude → height-band
filter (so the floor drops out) → keep the dominant horizontal plane → threshold on planar area. It
runs against the **simulated** cloud, so the same detector you validate in sim is the one that runs on
the robot. **Eye-calibrate** the z-band against the rendered cloud when wiring a new robot — the band
is mount-height-dependent.

## Per-robot, always

The tilt and occlusion sectors are **robot-specific** — they depend on the head/crown geometry. The
descriptor carries the measured values; if you onboard a new robot or a new head, re-run the `*_sight`
characterization (`discover-robot`) and the new descriptor flows straight through here. The
`calibration.reference_media` paths in the descriptor are the "what a calibrated envelope looks like"
images to check the sim against by eye.

## Hand-off

The mesh prim paths you pass are the static scene meshes the LiDAR casts against (ground, bed,
furniture) — they come from the scene built in `stage-isaac-rl-env`. Build the robot itself (free base,
deploy gains, locked DOFs) with `stage-isaac-freebase` first.
