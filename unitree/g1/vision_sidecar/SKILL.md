---
name: unitree-g1-vision-sidecar
description: >-
  Run accelerated GPU vision inference (DINOv2 ViT embeddings) for the Unitree G1 as a containerized
  sidecar over local RPC, so the host unitree_deploy env stays CPU-torch-only. Use when a consumer
  downstream of the G1's camera frames needs GPU-accelerated inference (e.g. DINOv2 features) and you
  don't want to install GPU torch into the robot's deploy env. Containerized service; pairs with
  unitree-g1-sense-depth's RGB stream.
metadata:
  tags: [unitree-g1, vision, dinov2, gpu, inference, sidecar, container, embeddings]
---

# Unitree G1 — GPU vision sidecar

A containerized GPU inference service (DINOv2 ViT embeddings) reachable over local RPC. It keeps GPU
torch **out** of the host `unitree_deploy` env — host consumers stay CPU-torch-only and send frames to
the sidecar when they need accelerated inference. Install, the service unit, and the RPC contract are in
**[`README.md`](README.md)** — this skill is the agent entry point.

> **Verified on hardware (2026-06-10):** built from the in-repo Dockerfile as
> `robotics-connect/vision-sidecar:0.1`, runs on the Jetson Orin **GPU** (`device=cuda`), and encodes a
> 480×640 RGB frame to the correct **384-d** DINOv2 CLS token at **~27.5 ms/frame** (~10× the CPU path).
> See [`VERIFICATION.md`](../../../VERIFICATION.md).

## When to use

- A consumer downstream of the G1's RGB frames ([`unitree-g1-sense-depth`](../depth_camera_sight/SKILL.md))
  needs **GPU-accelerated** inference (DINOv2 features) and you want to avoid installing GPU torch into
  the robot's deploy env.
- Pattern: send frames through the sidecar rather than co-locating a heavy model in `unitree_deploy`.
  `--cpu` on a consumer forces the in-process fallback.

## Try it

```bash
WITH_SIDECAR=1 bash ../install/install.sh    # installs the ~12 GB GPU sidecar
bash install.sh                              # build + start the container + systemd unit
```

See [`README.md`](README.md) for the Dockerfile, the `robotics-connect-vision-sidecar.service` unit, and
the RPC interface.
