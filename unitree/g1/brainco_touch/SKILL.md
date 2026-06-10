---
name: unitree-g1-hands
description: >-
  Drive and read the Unitree G1 EDU's Brainco 5-finger hands — open/close each digit, read fingertip
  touch and proximity — over the Modbus→TCP JSON bridge. Use when you need to command the G1's hands or
  read its tactile sensing, or to characterize the hand morphology (5 fingers, 6 motors) for a robot
  descriptor. The real robot's hand is Brainco (the sim uses Inspire — match finger count, not brand).
  Verified live on a G1 EDU (10/10 fingertips touch + proximity).
metadata:
  tags: [unitree-g1, hands, brainco, touch, proximity, tactile, grasp, modbus]
---

# Unitree G1 — Brainco 5-finger hands

The Brainco Revo2 hands as a Modbus→TCP JSON bridge: command 6 motors per hand (0=open … 1=closed), read
fingertip touch and proximity. Full install, the wire protocol, and the USB/digit/sensor mapping tables
are in **[`README.md`](README.md)** — this skill is the agent entry point.

## When to use

- **Command** the hands (`{"cmd":"set","left":[…6…],"right":[…6…]}`) or **read** touch + proximity
  (`{"cmd":"get"}`) on `127.0.0.1:9877`.
- Characterize the **hand morphology** for `discover-robot`.

## What it feeds the descriptor

| Descriptor field | Value |
|---|---|
| `hands.model` | Brainco Revo2 (rubber 5-finger) |
| `hands.fingers` / `hands.dof` | 5 fingers / 6 motors per hand (incl. the lateral thumb) |
| `hands.tactile` | true — 10/10 fingertips touch + proximity (verified) |
| `hands.control` | Modbus over a single FTDI FT4232H quad USB-UART → TCP JSON bridge |

## Real-to-sim: Brainco ↔ Inspire

The real robot has **Brainco** 5-finger hands; the Isaac G1 sim asset uses **Inspire** 5-finger hands.
`discover-robot` records this as `sim_asset.sim_real_reconciliation.hand_substitution` — the rule is
**match finger count, not brand** (the Isaac default Dex3 3-finger was rejected for that reason). The
body policy controls only the body joints, never the fingers, so the Brainco↔Inspire difference does not
enter the trained reach policy.

## Gotchas (the USB-port trap)

Both hands are channels of **one** FTDI quad chip, so VID/PID **and serial are identical** across all
four `ttyUSB*` ports. A hand can only be identified by Modbus probe (left `0x7e`, right `0x7f`), and the
port assignment **drifts** across robots/reboots — always probe, never hard-assume.

## Try it (on the robot)

```bash
bash install_brainco_touch.sh           # starts the bridge on 127.0.0.1:9877
python smoke_test.py                    # confirms touch is live
```

See [`README.md`](README.md) for the full mapping tables and the proximity-decode detail.
