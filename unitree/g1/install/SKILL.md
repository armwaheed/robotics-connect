---
name: unitree-g1-install
description: >-
  Deploy, uninstall, or build an offline bundle of the robotics-connect stack on a Unitree G1 EDU. Use
  when bringing the perception + hand-control stack up on the robot (or removing it): the installer
  deploys to /home/unitree/robotics-connect, creates a dedicated conda env cloned from the factory
  unitree_deploy, and installs a shell activation hook. Idempotent and fully reversible; includes a
  first-boot one-shot (WiFi + env + install) and an offline-bundle workflow for air-gapped robots.
metadata:
  tags: [unitree-g1, install, deploy, uninstall, offline-bundle, conda, on-robot]
---

# Unitree G1 — install / deploy the stack

On-robot deploy, uninstall, and offline-bundle of the `robotics-connect` stack. The full guide (what it
does/doesn't touch, the offline workflow, the conda-env clone) is in **[`README.md`](README.md)** — this
skill is the agent entry point.

## When to use

- **Bring up** the perception + hand-control stack on a G1 EDU, or **remove** it cleanly.
- **First-boot** a fresh robot (WiFi + env + install in one shot), or build an **offline bundle** for an
  air-gapped robot.

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
