---
name: setup-dgx-spark
description: >-
  Bring up Isaac Sim + Isaac Lab on an NVIDIA DGX Spark (GB10, aarch64) and avoid the non-obvious
  gotchas that cost hours. Use when setting up the host for an Isaac Sim RL job on a Spark (or any
  aarch64/GB10 box). Captures the must-dos: Isaac Sim built from source (no prebuilt aarch64 binary),
  the mandatory libgomp LD_PRELOAD, running scripts from the IsaacLab directory, the rsl_rl
  deprecation shim, no onnxruntime GPU provider (lift ONNX to torch), the Fabric render sync, headless
  eval video, and scaling num_envs on 128 GB unified memory. Sourceable env script included.
metadata:
  tags: [dgx-spark, gb10, aarch64, isaac-sim, isaac-lab, setup, ld-preload, onnxruntime, rsl-rl]
---

# Set up the DGX Spark for Isaac Sim RL

The DGX Spark (GB10 Grace-Blackwell, **aarch64**, sm_121, 128 GB unified, CUDA 13) runs Isaac Sim RL
well — but several steps are non-obvious and silently fatal if missed. This skill captures them. The
baseline is the [Arm Learning Path](https://learn.arm.com/learning-paths/laptops-and-desktops/dgx_spark_isaac_robotics/);
the gotchas below are what that baseline doesn't tell you.

**Source the env first:** [`scripts/spark_env.sh`](scripts/spark_env.sh) sets the libgomp preload and
defines an `isaaclab` wrapper that always runs from the IsaacLab directory.

```bash
source scripts/spark_env.sh
isaaclab -p path/to/script.py --headless ...
```

> Networking to the *robot* (host ↔ robot subnet) is a different concern — see the
> [`unitree/g1/connect`](../../unitree/g1/connect/SKILL.md) skill / `configure_spark.sh`. This skill is
> the *host's Isaac stack*.

## The platform

| Component | Detail |
|---|---|
| Machine | DGX Spark — GB10, **aarch64**, sm_121, 128 GB unified memory, CUDA 13 |
| Isaac Sim | **5.1.0, built from source** — no prebuilt aarch64 binary/container exists |
| Isaac Lab | **2.3.2** (`./isaaclab.sh --install`), symlinked to the source Sim build |
| RL library | **rsl-rl-lib 5.0.1** (bundled with Isaac Lab 2.3.2) |
| PyTorch | cu13 build; GB10 is sm_121 (newer than torch's max advertised arch) → warns but runs |

## The gotchas (each one cost real time)

1. **Build Isaac Sim from source.** Prebuilt containers/binaries target x86_64; none exist for aarch64.
   The native source build of Isaac Sim 5.1.0 is the working path; Isaac Lab 2.3.2 symlinks to it.

2. **`LD_PRELOAD` libgomp before *every* Isaac run** (aarch64 caveat):
   ```bash
   export LD_PRELOAD="$LD_PRELOAD:/lib/aarch64-linux-gnu/libgomp.so.1"
   ```
   `spark_env.sh` does this idempotently. Without it, Isaac crashes on import.

3. **Run from the `IsaacLab` directory.** `./isaaclab.sh` is a **relative** launcher — `cd`-ing into a
   subdir and running it gives a silent `exit 127`. Use the absolute `…/IsaacLab/isaaclab.sh`, or the
   `isaaclab` wrapper from `spark_env.sh`, from anywhere.

4. **No onnxruntime GPU execution provider** on aarch64/GB10. Don't fight it — **lift the exported ONNX
   MLP into a torch stack on `cuda`** (parity ~3e-6). See `deploy-policy`'s `load_onnx_mlp_to_torch`.
   CPU inference is the last resort, not the plan.

5. **`rsl_rl KeyError: 'class_name'`.** Isaac Lab 2.3.2 bundles rsl-rl-lib 5.x (new `actor`/`critic` +
   `obs_groups` schema); the official Unitree trainer skips Isaac Lab's deprecation shim. Fix = call
   `handle_deprecated_rsl_rl_cfg(...)` (2 lines, already in `stage-isaac-rl-env`'s train.py/play.py).
   **Not** a version/Docker problem (unitree_rl_lab#115).

6. **GB10 is sm_121**, newer than torch's max advertised arch → it warns but runs. Don't downgrade torch
   chasing the warning.

7. **Fabric render sync for articulations + particle cloth.** Use `use_fabric=True` so the robot
   articulation renders its true motion (Isaac Lab only pushes link poses to the renderer with Fabric
   on). PhysX does **not** sync particle-cloth deformation to Fabric → **blit the live tensor cloth-view
   points into the Fabric mesh points each render** (mesh updates cross Fabric; point-instancer ones
   don't). See the `isaac-particle-cloth` note in armwaheed/robots#2.

8. **Headless eval video.** `gymnasium.RecordVideo` doesn't capture Isaac Lab vec envs headless
   (IsaacLab#875) → use an in-scene `CameraCfg` + per-step rgb read + ffmpeg (the path `play.py` uses).

9. **128 GB unified memory** runs **2048 parallel humanoids on ~4 GB** — scale `num_envs` freely; the
   memory is not the constraint here.

## Sanity check

```bash
source scripts/spark_env.sh
isaaclab -p .../stage-isaac-freebase/scripts/check_spawn.py \
    --descriptor .../discover-robot/descriptors/unitree_g1_29dof.json
```

If `check_spawn` prints `OVERALL: PASS`, the host stack is healthy and you can train.

## Performance — the GB10 GPU "stuck low-power" trap (check this FIRST when training is slow)

The GB10's GPU power controller can get **trapped in a low-power state** (triggered by a crash, a failed
load, sleep/wake, or many heavy back-to-back jobs). Symptom: under load `nvidia-smi` shows **high util
(~80–96%) but the SM clock pinned to a small fraction of max** (we saw **507 MHz vs a 2418 MHz app clock /
3003 MHz max**) at only a few watts and a cool ~40 °C — i.e. **not thermal, not CPU-bound, not contention.**
It silently makes training **~3–5× slower** (a 2048-env whole-body job went 67 min → 213 min).

Diagnose (the clock-vs-max gap is the tell — Performance State still reads `P0` and Clock Event Reasons read
"Not Active", which is misleading):
```bash
nvidia-smi --query-gpu=utilization.gpu,clocks.sm,clocks.max.sm,temperature.gpu,power.draw --format=csv
# verify under real load: torch matmul burn while sampling — healthy = clocks.sm boosts to ~2000-3000 MHz
```
**Fix: a full AC power cycle is the ONLY one** — shut down → **unplug the power adapter from the wall** (≥60 s;
a normal reboot does NOT clear it — residual power keeps the controller latched) → replug → boot. After the
cycle the GPU boosts to its full clock (~2500 MHz / ~100 W under load). Diagnostic tool:
[spark-doctor](https://github.com/joeynyc/spark-doctor). Refs:
[NVIDIA forum 370304](https://forums.developer.nvidia.com/t/dgx-spark-grace-blackwell-gb10-performance-drop-gpu-trapped-in-15w-650mhz-loop-with-50-c-artificial-t-limit-temp/370304),
[step-by-step fix](https://dredyson.com/fix-dgx-spark-performance-degradation-gpu-power-draw-issue-in-under-5-minutes-actually-works-a-complete-step-by-step-beginners-guide-to-resolving-the-14w-power-cap-low-token-rate-and-stuck-pe/).
**Lesson: check the clock-vs-max before blaming a config change.**
