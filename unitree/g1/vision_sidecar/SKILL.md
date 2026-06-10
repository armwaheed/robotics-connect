---
name: unitree-g1-vision-sidecar
description: >-
  Run accelerated GPU vision inference (DINOv2 ViT embeddings) for the Unitree G1 as a containerized
  sidecar over local RPC, so the host unitree_deploy env stays CPU-torch-only. Use when a consumer
  downstream of the G1's camera frames needs GPU-accelerated inference (e.g. DINOv2 features) and you
  don't want to install GPU torch into the robot's deploy env. The sidecar is placeable + targetable +
  device-selectable (VISION_SIDECAR_HOST/PORT/DEVICE) so it coexists on an alternate port, runs one per
  GPU, or runs on a peripheral expansion-port node (e.g. a Jetson Thor) — composing with the descriptor
  compute block. Pairs with unitree-g1-sense-depth's RGB stream.
metadata:
  tags: [unitree-g1, vision, dinov2, gpu, inference, sidecar, container, embeddings, multi-gpu, ports]
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

## Placement & coexistence (multi-GPU / peripheral / alternate port)

The sidecar is **placeable, targetable, and device-selectable** — the GPU half of the "any humanoid"
discoverability theme. It composes with the descriptor's
[`compute`](../../../skills/discover-robot/schema/robot_descriptor.schema.json) block (each accelerator
node carries a `host` and `device`):

- **Enumerate** a node's accelerators: `docker run --rm --runtime=nvidia robotics-connect/vision-sidecar:0.1 --topology`.
- **Alternate port (coexistence)** — if `9878` is held by another sidecar/service, **don't fight for it**;
  run on another port and target it: `-e VISION_SIDECAR_PORT=9879`. (Used live to verify the sidecar on a
  G1 whose `9878` was occupied.)
- **One per GPU** on a multi-GPU node: a second instance with `-e VISION_SIDECAR_DEVICE=cuda:1 -e VISION_SIDECAR_PORT=9879`.
- **Peripheral GPU** (e.g. a Jetson Thor on the expansion port): run the sidecar **on the Thor**, record it
  as a `compute` node (`location: expansion`, `host: <thor-ip>`); clients target `<thor-ip>:9878`. No code
  change — only the descriptor's `compute.host` differs.

The host-side client reads the `host:port` to target **from the descriptor's compute block**, so GPU
placement is data-driven. See [`README.md`](README.md) § *Multiple GPUs, peripheral nodes & port coexistence*.

## Try it

```bash
WITH_SIDECAR=1 bash ../install/install.sh    # installs the ~12 GB GPU sidecar
bash install.sh                              # build + start the container + systemd unit
```

See [`README.md`](README.md) for the Dockerfile, the `robotics-connect-vision-sidecar.service` unit, and
the RPC interface.
