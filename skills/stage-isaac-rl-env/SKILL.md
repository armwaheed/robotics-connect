---
name: stage-isaac-rl-env
description: >-
  Stand up a runnable manager-based Isaac Lab RL env for a whole-body reach / loco-manipulation task,
  from a robot descriptor. Use after stage-isaac-freebase to produce a trainable + eval-able job. Brings
  the proven bed-reach recipe (station-keeping so the robot reaches by leaning and stays planted,
  FALCON-style grip-slip force load, SYMDEX same-side ambidexterity, termination-dominant reward) with
  the action joint set and EE links read from the descriptor (so a 23-DOF robot trains a 23-DOF policy
  and a 29-DOF robot commands all 29). Includes train.py / play.py with the aarch64 + rsl_rl + headless
  -video gotchas baked in. Eye-verified on the Unitree G1.
metadata:
  tags: [isaac-lab, reinforcement-learning, rl-env, loco-manipulation, ppo, reward-shaping, symdex, falcon]
---

# Stage the RL env — the whole-body reach template

A robot-agnostic scaffold of the bed-reach pattern from armwaheed/robots#2, generalized to a
task-target command and driven by the [robot descriptor](../discover-robot/SKILL.md). Given a free-base
robot (`stage-isaac-freebase`) it produces a **runnable** Isaac Lab RL job.

Scripts:

| Script | Role |
|---|---|
| [`scripts/reach_env_cfg.py`](scripts/reach_env_cfg.py) | The env: scene + task-surface obstacle, base-frame target command, action = descriptor's action joints, 151-D-style obs, the reward/termination recipe. |
| [`scripts/mdp.py`](scripts/mdp.py) | Custom terms: station-keeping, same-side reach, idle-arm, grip-slip force (lifted verbatim — already robot-agnostic). |
| [`scripts/agents.py`](scripts/agents.py) | PPO runner cfg (MLP [512,256,128] ELU). |
| [`scripts/train.py`](scripts/train.py) · [`scripts/play.py`](scripts/play.py) | Train → export JIT+ONNX; eval → mp4 (verify by eye). |

## What makes the policy work (the reward recipe)

Each piece is an application of reused research; together they are why the robot reaches **planted**
without toppling:

| Term | Role | From |
|---|---|---|
| `reach_coarse` / `reach_fine` (tanh std 0.20 / 0.06) + `reach_l2` | Shape then sharpen the same-side hand → target | locomotion reward shaping |
| **`base_anchor` (xy drift, −2.0)** | **Station-keeping** — reach by leaning, stay planted. THE fix for the deploy walk-off topple. | standard reward shaping |
| `termination_penalty` (−200, dominant) + `upright` + `base_height` + `feet_slide` | Don't fall; stay vertical; plant the feet | — |
| **grip-slip `ee_load`** (0–35 N, toggles on/off every 1–2.5 s) | Absorb a sudden hand-load change without toppling | **FALCON** force-adaptive WBC |
| **same-side reach + `idle_arm`** | **Ambidexterity** — lead with the hand on the target's side; idle arm hangs naturally | **SYMDEX** (arXiv:2505.05287) |
| `action_rate` / `dof_acc` / `dof_torques` / `dof_pos_limits` | Smooth, hardware-able motion | — |

**Ambidexterity needs no observation change** — the same-side reward/idle-arm/force terms read the
active hand from the command's lateral sign, so the deployed policy stays a drop-in (`deploy-policy`).

## Descriptor-driven, so it generalizes

The env reads the **action joint set** (`action_joints(descriptor)`) and **EE links** from the
descriptor, selected via env vars:

```bash
export ROBOTICS_CONNECT_DESCRIPTOR=.../discover-robot/descriptors/unitree_g1_29dof.json
export ROBOTICS_CONNECT_MOBILE_USD=.../stage-isaac-freebase/assets/g1_inspire_mobile.usd
```

So the **same env** trains a 29-DOF-action policy for a full G1, or a 23-DOF-action policy for the G1
EDU (the 6 locked joints are excluded from the action and held at default — see `stage-isaac-freebase`).
The reward joint-groups (hips/waist/ankle/knee/arm regexes) resolve against the articulation and are the
G1 worked example; adapt them for a robot with a very different joint tree.

## Run it

```bash
cd ~/workspaces/git/IsaacLab
export LD_PRELOAD="$LD_PRELOAD:/lib/aarch64-linux-gnu/libgomp.so.1"   # aarch64 must-do

# train (~30-65 min on one GB10; 2048 envs fit in ~4 GB of the 128 GB unified memory)
./isaaclab.sh -p .../scripts/train.py --headless --num_envs 2048 --max_iterations 1000

# eval → mp4 (VERIFY BY EYE — never trust reward telemetry alone)
./isaaclab.sh -p .../scripts/play.py --headless --enable_cameras --video --num_envs 4 \
    --video_length 350 --checkpoint .../logs/reach_wbc/<run>/model_999.pt
```

## What good looks like (the reference standard)

Convergence: 2048 parallel envs, ~1000 PPO iterations, hand→target error **~37 cm → ~6 cm**, base drift
small and stable (planted), falls rare even with the grip-slip load toggling. If your curve doesn't take
this shape, the env / gains / reward are wrong. The reference media live in
[`assets/media/rl/`](../../assets/media/) — `convergence.png` (the curve) and `ambidextrous_eval.mp4`
(both-handed, balanced, no topple). **These are the "what good looks like" references** an agent matches
a novel humanoid's training against.

## Gotchas baked in

- **rsl_rl `KeyError: 'class_name'`** — Isaac Lab 2.3.2 ships rsl-rl-lib 5.x; `train.py`/`play.py` call
  `handle_deprecated_rsl_rl_cfg(...)`. Not a version/Docker problem.
- **Headless eval video** — `gymnasium.RecordVideo` doesn't capture Isaac vec envs headless; `play.py`
  reads an in-scene `Camera` and ffmpeg-encodes.
- **Run from the `IsaacLab` directory**; `LD_PRELOAD` libgomp first. See `setup-dgx-spark`.

## Hand-off

Train → eval (verify by eye) → `deploy-policy` to run the exported policy out of the RL harness.

## Sim-to-real obs design — asymmetric actor-critic for a deployable observation

A policy can only use observations the **real robot can produce**. The G1's **base linear velocity** is not
reliably observable on hardware (the deploy used a noisy leg-kinematics odom estimate), so it must be excluded
from the **actor**. But dropping it from the **critic** too **starves the value function** — in this project
`reach_coarse` peaked at 0.39 then *regressed* to 0.29 (ep_len ~210/400, topples ~30%).

**Fix: asymmetric actor-critic.** Define two observation groups — `policy` (actor, deployable: no
`base_lin_vel`) and `critic` (privileged, sim-only: includes `base_lin_vel` + any other privileged terms,
clean/no-noise). rsl-rl 5.x uses the `critic` group for the value net automatically (it warns and falls back
to `policy` only if `critic` is absent). The deployed actor stays small; the critic keeps the velocity signal.
Result here: `reach_coarse` **0.59 monotonic**, ep_len **394/400**, robust — recovering nearly all the
performance lost by dropping `base_lin_vel`, while the actor stays hardware-deployable (82-D).

```python
class ObservationsCfg:
    class PolicyCfg(ObsGroup):   # ACTOR — deployable; NO base_lin_vel
        base_ang_vel = ...; projected_gravity = ...; <command> = ...; joint_pos = ...; joint_vel = ...; actions = ...
    class CriticCfg(ObsGroup):   # CRITIC — sim-only privileged; KEEPS base_lin_vel; enable_corruption=False
        base_lin_vel = ...; base_ang_vel = ...; projected_gravity = ...; <command> = ...; joint_pos = ...; joint_vel = ...; actions = ...
    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()
```

Also: judge convergence by `reach_coarse` + ep-len + the **rendered eval**, not the regularizer-dominated mean
reward; keep reward terms few (<10, vs heavy 20+-term shaping). Refs: [FALCON (arXiv 2505.06776)](https://arxiv.org/abs/2505.06776),
[Isaac Lab sim-to-real / privileged obs](https://isaac-sim.github.io/IsaacLab/main/source/experimental-features/newton-physics-integration/sim-to-real.html).
