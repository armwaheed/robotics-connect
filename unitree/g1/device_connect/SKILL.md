---
name: unitree-g1-device-connect
description: >-
  The Unitree G1 DRIVER for Arm's open-source Device Connect framework (https://github.com/arm/device-connect)
  — Device Connect itself is the agent-orchestration skill; this module only adapts the G1 to it. It
  registers the robot as a DEVICE on the fabric so an orchestrator (or a human agent) can drive it over
  the network: SAY text through the chest speaker, REQUEST HELP from a human partner (grounded
  yes/no/multiple-choice reply), and read STATUS — with help_requested / help_answered events on the
  dashboard. The ROBOT side of the human-in-the-loop loop (the bed-making G1 that asks a person when
  it's stuck); the human side is human_agent, the two-env bridge is bootstrap-device-connect-env. Use
  when the robot must be a first-class Device Connect device, not a local script.
metadata:
  tags: [unitree-g1, deviceconnect, driver, human-in-the-loop, orchestration, voice, help-seeking]
---

# Unitree G1 — driver for Arm Device Connect

> **Device Connect is the skill.** The agent-orchestration framework — `DeviceRuntime`, the
> `@rpc`/`@emit` decorators, the NATS/JWT device registration, the dashboard — is Arm's open-source
> **[device-connect](https://github.com/arm/device-connect)** (`device-connect-edge`). This module is
> **only the Unitree G1 driver** for it: it makes the robot a *device* Device Connect can call. It does
> not reimplement Device Connect and is not a substitute for it.

Register the G1 as that device with [`g1_agent.py`](g1_agent.py) (`G1AgentDriver`,
`device_type = "unitree_g1"`); the chest-speaker side runs through [`g1_speak.py`](g1_speak.py), via the
shared sidecar in [`../../../lib/device_connect_sidecar.py`](../../../lib/device_connect_sidecar.py) and
the two-env bridge from
[`skills/bootstrap-device-connect-env`](../../../skills/bootstrap-device-connect-env/SKILL.md)
(`device-connect-edge` needs Python ≥3.11 while the G1 speaker SDK env is 3.10).

## When to use

- The robot must be **reachable over Device Connect** (visible on the dashboard, callable by an
  orchestrator or another agent) — not just driven by a local Python loop.
- It needs to **ask a human for help and act on the answer** — `request_help` invokes the paired
  human agent and returns the grounded reply.
- You want robot ↔ human dialogue to be the **source of truth** (RPC + events on the fabric); the
  audio is a swappable presentation layer.

## Interface

| Kind | Name | What it does |
|---|---|---|
| `@rpc` | `say(text)` | speak `text` through the chest speaker (`g1_speak.py` over the DDS voice service) |
| `@rpc` | `request_help(question, choices="yesno", listen_seconds=8)` | ask the human partner, return the grounded `{choice, transcript}` |
| `@rpc` | `get_status()` | name, device_type, DDS interface, current state |
| `@emit` | `help_requested(question, choices)` | the robot asked for help |
| `@emit` | `help_answered(question, choice, transcript)` | the human answered |

## How to use

```bash
# On the robot (the dc-repro py3.11 env), with a NATS creds file:
python g1_agent.py --creds /path/to/<robot>.creds.json --name "G1 EDU"
```

The device registers on the hosted fabric (`nats://fabric.deviceconnect.dev:4222`); `request_help`
routes `robot → human_agent → grounded reply → robot`, emitting `help_requested` / `help_answered` so
the exchange is visible on the dashboard. Pair with the **human_agent** (Bluetooth-headset human) for
the full out-loud loop. Off-fabric, the shared sidecar degrades to an offline self-test.

## Notes

- `say` drives the verified `AudioClient` chest-speaker path; the on-board **microphone is not a
  developer interface**, which is *why* the human is routed in as a Device Connect agent rather than
  read from the robot's mic (see [`../voice`](../voice/SKILL.md)).
- Run **one** DDS-initialising component per process; the agent and any locomotion/voice driver in the
  same process should share the channel factory.

## Built on Arm Device Connect

The runtime is **Arm's open-source [device-connect](https://github.com/arm/device-connect)**
(`pip install device-connect-edge`, pinned to **0.2.4**) — `DeviceRuntime`, the `DeviceDriver` base,
`@rpc`/`@emit`, the identity/status types, and the NATS/JWT fabric registration all come from there.
`G1AgentDriver` only adds the G1-specific RPCs/events and the chest-speaker bridge. The contract is
**owned upstream**; the single source of truth for the wrapper is
[`lib/README.md`](../../../lib/README.md). If you bump the version, re-check `DeviceRuntime(...)` kwargs
and the `drivers`/`types` import paths.
