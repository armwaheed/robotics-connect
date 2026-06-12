---
name: bootstrap-device-connect-env
description: >-
  Get Device Connect running on a robot/compute host without breaking the vendor SDK, by resolving
  the Python-version collision agentically. device-connect-edge requires Python >=3.11, but most
  humanoid SDKs pin an older Python (Unitree G1 = 3.10, ROS 2 Humble / Jetson stacks = 3.8/3.10)
  with native deps (CycloneDDS, vendor wheels) that are painful to rebuild — so installing Device
  Connect into the SDK env fails ("No matching distribution") and forcing the SDK onto 3.11 bricks a
  working install. Use BEFORE running any Device Connect sidecar/driver on a new humanoid (or the
  G1): probe the host's Python envs, decide UNIFIED vs the two-env BRIDGE, bootstrap a clean >=3.11
  env, and verify. Generalizes to any humanoid; G1-specific notes included.
metadata:
  tags: [device-connect, conda, venv, python, environment, bootstrap, humanoid, unitree-g1, sidecar, dds]
---

# Bootstrap a Device Connect environment (without breaking the vendor SDK)

`device-connect-edge` needs **Python ≥ 3.11**. The robot's SDK almost never does. That single
collision is the #1 reason a Device Connect bring-up stalls on a new humanoid — and the wrong fix
(rebuild the SDK on 3.11) bricks a working robot. This skill resolves it **agentically**: it doesn't
hand you one env recipe, it teaches the agent to **diagnose the host and pick the right resolution**,
because the answer differs per robot.

> **A challenge to the framing.** The instinct is "make a conda environment." But provisioning an env
> isn't the problem — a **version-floor collision** is (DC's `>=3.11` floor vs the SDK's pinned
> ceiling), and a one-env bootstrap *fails on exactly the robots that need it* (it hits the
> native-dep rebuild wall). So this skill (a) is conda-**or**-venv, (b) decides UNIFIED vs BRIDGED
> from probed facts rather than assuming, and (c) treats the **two-env bridge as the correct
> architecture for a vendor-pinned SDK, not a hack**. Build that, not "an env."

## Procedure (agentic — reason from what you find)

### 1. Probe the host
```bash
python3 scripts/probe_dc_env.py --sdk-module <SDK_IMPORT_NAME>   # e.g. unitree_sdk2py
```
It enumerates every interpreter (conda envs, venvs, system), reports each one's Python version,
whether the **SDK** imports there, whether **device-connect-edge** imports there, and whether it
meets the ≥3.11 floor — then prints a recommendation. (Don't know the SDK import name? Run it with
no `--sdk-module` to try the common ones, or check the descriptor / `pip show`.)

### 2. Decide
- **UNIFIED** — one env already has the SDK *and* Python ≥3.11 → just
  `pip install device-connect-edge` there. (Rare for vendor SDKs; common for pure-Python robots.)
- **BRIDGED** (the usual vendor-SDK case) — the SDK env is < 3.11 → **keep it untouched**, create a
  clean ≥3.11 env for Device Connect, and have the DC sidecar **delegate hardware calls (speak/move/
  read) to the SDK env via subprocess/IPC**. See [`references/two-env-bridge.md`](references/two-env-bridge.md).
- **Do NOT** `pip install --ignore-requires-python` device-connect-edge into the SDK env, and do NOT
  rebuild the SDK/CycloneDDS on 3.11. Either can break a working robot for no real gain.

### 3. Bootstrap the Device Connect env (only if BRIDGED, or no ≥3.11 env exists)
```bash
bash scripts/bootstrap_dc_env.sh                 # conda env (or venv) "device-connect" + DC packages
# audio human-agent host? add the ASR/TTS deps:
DC_EXTRA_PKGS="faster-whisper piper-tts" bash scripts/bootstrap_dc_env.sh
```
Conda-or-venv agnostic, idempotent. Never touches the SDK env.

### 4. Verify end-to-end
- `device_connect_edge` imports in the DC env; the SDK still imports in its env (unchanged).
- A smoke DeviceRuntime registers on the fabric; one delegated hardware call (e.g. `say`) works.

## G1 EDU specifics (verified)
- SDK env `robotics-connect` / `unitree_deploy` = **Python 3.10** (`unitree_sdk2py` editable +
  CycloneDDS on `eth0`). Device Connect env `dc-repro` = **Python 3.11**. → **BRIDGED.**
- The chest speaker is DDS on the robot's internal `eth0` (a separate compute host can't reach it),
  so the speaking CLI must run **on the robot, in the SDK env** — the sidecar (3.11) subprocesses to
  it. This is the `rabia_agent.py` / `rabia_speak.py` split in `unitree/g1/device_connect/`.

## Files
- [`scripts/probe_dc_env.py`](scripts/probe_dc_env.py) — agentic env probe + UNIFIED/BRIDGED recommendation (pure stdlib).
- [`scripts/bootstrap_dc_env.sh`](scripts/bootstrap_dc_env.sh) — create a clean ≥3.11 DC env (conda or venv), idempotent.
- [`references/two-env-bridge.md`](references/two-env-bridge.md) — the bridge architecture + the Rabia worked example.
