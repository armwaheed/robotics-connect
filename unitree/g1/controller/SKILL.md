---
name: unitree-g1-controller
description: >-
  Read the Unitree G1's handheld controller and turn ANY button press into a clean software abort of
  an autonomous routine. Parses LowState.wireless_remote (uint16 button bitmask + analog sticks),
  arms once buttons are released, then latches an abort that wires into the robot-agnostic
  LocomotionController.set_abort_source so walk_to/turn_to stop and hold balance on a press. Use when
  a human supervises a hands-off routine and needs a one-button halt from the controller they hold.
  NOT an emergency stop — the firmware damping and physical power/e-stop are the real stop.
metadata:
  tags: [unitree-g1, controller, remote, abort, safety, teleop, locomotion, supervision]
---

# Unitree G1 — Controller (abort from the handheld remote)

Turn the controller a supervisor is already holding into a clean abort for an autonomous routine.
The API, the byte layout, the arming/latch behaviour, and the **read-only dry test** are in
**[`README.md`](README.md)** — this skill is the entry point. Module: [`g1_remote.py`](g1_remote.py).

> **Not the e-stop.** This software watcher depends on the process + DDS being alive. The
> controller's firmware damping and the robot's physical power/e-stop are the real emergency stop —
> document that wherever this is used, and never present the any-button watcher as the safety net.

## When to use

- A human supervises a **hands-off routine** (e.g. the bed-making walk-to-bed) and needs to **halt it
  from the controller** without a specific button combo.
- You're wiring a routine's **abort source** — the watcher's `aborted()` plugs straight into
  `LocomotionController.set_abort_source` (in `lib/locomotion.py`), so the walk helpers stop + hold.
- You need the raw controller state (`pressed()`, `sticks()`) for diagnostics or teleop.

## How to use

```python
import sys; sys.path.insert(0, "unitree/g1/controller")
from g1_remote import G1Remote

remote = G1Remote(iface="eth0", init_dds=False)   # if the locomotion driver already inited DDS
remote.connect(); remote.wait_until_armed()
loco.set_abort_source(remote.aborted)             # walk_to/turn_to return "aborted" on any press
while not remote.aborted() and steps_left:        # also poll between routine steps
    run_next_step()
```

**Always dry-test first** (read-only, no motion): `python g1_remote.py --iface eth0`, press every
button, confirm `ABORTED=True` latches. An unverified abort is worse than none.

## Safety contract

Abort = **stop motion + hold balance** (not collapse). For a true emergency — a fall in progress, a
flailing arm — use the controller's firmware damping or cut power. Two tiers: this watcher
de-escalates the routine; the hardware stop is the kill.
