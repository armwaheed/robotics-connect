---
name: unitree-g1-install
description: >-
  Bootstrap, self-verify, uninstall, or offline-bundle the Unitree G1 EDU capability stack on the robot.
  Use when bringing the perception + hand-control stack up on the robot, or to PROVE an install is good
  in one command (install.sh --verify runs a probe-based PASS/FAIL self-check of every sensor + both
  hands). This is the G1's on-robot BOOTSTRAP — independently deployable on its own, and the step-zero
  that discover-robot delegates to (it does not assume the stack is live; it verifies, then bootstraps).
  Idempotent and reversible; clones a dedicated conda env from factory unitree_deploy without mutating it.
metadata:
  tags: [unitree-g1, bootstrap, install, verify, deploy, uninstall, offline-bundle, conda, on-robot]
---

# Unitree G1 — bootstrap + self-verify the stack

The G1's on-robot **bootstrap**: deploy the capability stack, then prove it works. The full guide (what
it does/doesn't touch, the offline workflow, the conda-env clone) is in **[`README.md`](README.md)** —
this skill is the agent entry point.

> **Where this sits in the theme.** This bootstrap is **robot-scoped** and **independently deployable**
> (you can deploy the G1 stack with no agent involved). It is *also* the step-zero that the
> robot-agnostic [`discover-robot`](../../../skills/discover-robot/SKILL.md) skill **delegates** to:
> discovery calls `install.sh --verify` first and bootstraps only if the stack isn't live. The bootstrap
> is subordinate to discovery, not absorbed into it — so a different robot brings its own bootstrap and
> `discover-robot` stays generic.

## When to use

- **Verify** an install is healthy in one command (`install.sh --verify`) — the probe-based self-check.
- **Bring up** the perception + hand-control stack on a G1 EDU, or **remove** it cleanly.
- **First-boot** a fresh robot (WiFi + env + install in one shot), or build an **offline bundle** for an
  air-gapped robot.

## Self-verify (the one-command health check)

```bash
bash install/install.sh --verify        # PASS/FAIL scoreboard: install, arm_fk, camera, rgb, lidar, hands
bash install/install.sh --verify hands  # or a subset
```

It probes live state — the hand check **auto-detects** the ports by Modbus slave id (`0x7e`/`0x7f`),
never assuming `ttyUSB1/2`. Exit 0 iff every selected check passes. Validated live on a G1 EDU
(all 6 checks PASS). Pure software + read-only sensor reads — **no robot motion**.

## What it does (safe + reversible)

Deploys to `/home/unitree/robotics-connect/`, creates a dedicated `robotics-connect` conda env cloned
from the factory `unitree_deploy`, and installs a shell activation hook. **Idempotent** (re-running is a
no-op when already present) and **fully reversible** (`uninstall.sh` returns the robot to factory state
and removes the `robotics-connect-*` systemd units). It does **not** `apt install` anything — it builds
only against libraries already on the target image, so it's safe to run unattended without sudo prompts.

## Try it (on the robot)

```bash
bash install/install.sh                      # core stack
WITH_SIDECAR=1 bash install/install.sh       # also the ~12 GB GPU vision sidecar
SSID="YourNetwork" WIFI_PASS="..." bash install/first-boot.sh    # fresh-robot one-shot
bash /home/unitree/robotics-connect/install/uninstall.sh         # back to factory
```

The Brainco hand bridge installs separately (`brainco_touch/install_brainco_touch.sh`, targets the
`g1brainco` env) — see [`unitree-g1-hands`](../brainco_touch/SKILL.md). See [`README.md`](README.md) for
the offline-bundle steps and the deploy gotchas (use `scp`, not `rsync`).
