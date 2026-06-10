# Validation media — reference standards

This media is **not decoration.** For an agent discovering a robot or applying a skill to a novel one,
these are the **reference standards**: what a calibrated sensor's envelope looks like, what RL
convergence looks like, and what a good policy looks like **by eye** — this project's hard rule is
*verify by what a human sees, never by reward telemetry alone*. When you stage an Isaac RL job for a new
humanoid, match its training and eval against these; if the convergence curve doesn't take this shape or
the eval doesn't look like this, the env / gains / reward are wrong.

Lifted from the eye-verified armwaheed/robots#2 bed-making work.

## RL — what good looks like (`rl/`)

| File | What it is the reference for |
|---|---|
| `convergence.png` | PPO convergence on one GB10: hand→target error **~37 cm → ~6 cm**, reach reward saturating, base drift small + stable (planted). The shape a novel humanoid's training should match. |
| `ambidextrous_eval.mp4` | The current policy in isolation — **both-handed, balanced, no topple** (SYMDEX same-side reach). What a good whole-body reach policy looks like by eye. |
| `ambidextrous_reach.png` / `ambidextrous_lateral.png` | Reach over the bedside and the lateral (drag-direction) reach, ambidextrous. |
| `bed_pull_reach.png` / `bed_pull_lateral.png` | The planted bed-pull: feet planted outside the bed, leaning over to reach — station-keeping working. |
| `02_deep_reach.png` | Free-space deep reach (iteration 1) — balance-while-reach proven before the bedside additions. |
| `walk_in_arms_at_sides.png` | The walk-in stance: arms at the sides (the canonical Unitree pose), the neutral the reach policy is centred on for a clean walk→reach handoff. |
| `bedmaking_scene.png` | The end-to-end two-G1 bed-making scene. |

## Sensor calibration — the real envelope to match the sim to

These live with each robot-scoped capability (not duplicated here):

- **LiDAR** near-field + self-occlusions — [`unitree/g1/lidar_sight/images/`](../../unitree/g1/lidar_sight/images/)
- **RGB / depth** head-cam samples (51.29° down-tilt) — [`unitree/g1/depth_camera_sight/images/`](../../unitree/g1/depth_camera_sight/images/)

The descriptors point at these via each sensor's `calibration.reference_media`.
