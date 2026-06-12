---
name: human-agent
description: >-
  A DRIVER for Arm's open-source Device Connect framework (https://github.com/arm/device-connect) that
  registers a HUMAN as a device on the fabric — Device Connect is the agent-orchestration skill; this
  only adapts a person to it. Captures a Bluetooth headset, runs local ASR (faster-whisper), and grounds
  the spoken reply to expected choices (KnowNo-style MCQA), exposing ask() / notify() / presence() RPCs
  and a human_replied event. So a robot (e.g. the bed-making G1 via unitree/g1/device_connect) can ASK A
  PERSON FOR HELP over the fabric and act on the grounded answer. Robot-agnostic; run it on any host
  within Bluetooth range of the headset that can reach the fabric.
metadata:
  tags: [deviceconnect, human-in-the-loop, driver, headset, asr, grounding, help-seeking, robot-agnostic]
---

# Human Agent — a person as a Device Connect device

> **Device Connect is the skill.** The agent-orchestration framework — `DeviceRuntime`, the
> `@rpc`/`@emit` decorators, the NATS/JWT registration, the dashboard — is Arm's open-source
> **[device-connect](https://github.com/arm/device-connect)** (`device-connect-edge`). This module is
> **only a driver** for it: it registers a *human* (Bluetooth headset + local ASR) as a `human_agent`
> device so any robot on the fabric can call them. It does not reimplement Device Connect.

The matching **robot** side is [`unitree/g1/device_connect`](../unitree/g1/device_connect/SKILL.md); this
is the **human** side. Modules: [`human_agent.py`](human_agent.py) (`HumanAgentDriver`,
`device_type = "human_agent"`), headset I/O in [`bt_headset.py`](bt_headset.py), shared sidecar in
[`../lib/device_connect_sidecar.py`](../lib/device_connect_sidecar.py).

## When to use

- A robot needs to **ask a human for help** and act on the spoken reply, with the human represented on
  the fabric rather than hard-wired to the robot.
- The robot's own microphone isn't usable (e.g. the [G1's mic is not a developer interface](../unitree/g1/voice/SKILL.md))
  — route the human in over Device Connect instead.
- You want the human's answer as a fabric **event** (`human_replied`), visible on the dashboard.

## Interface

| Kind | Name | What it does |
|---|---|---|
| `@rpc` | `ask(question, choices="yesno", …)` | prompt the human, capture the headset, ASR, **ground** → `{choice, transcript}` |
| `@rpc` | `notify(message)` | one-way message to the human |
| `@rpc` | `presence()` | is the human available? |
| `@emit` | `human_replied(question, choice, transcript)` | emitted after every grounded answer |

## How to use

```bash
python human_agent.py --creds /path/to/<agent>.creds.json --name "Bluetooth Headset Human Agent"
```

Registers a `human_agent` device on the fabric; a robot's `request_help` then routes
`robot → human_agent.ask → grounded reply → robot`. ASR is pluggable (faster-whisper by default);
grounding maps free speech to one of the expected choices (KnowNo-style MCQA — "yeah, go ahead" → `yes`).
`bt_headset.py` auto-detects a connected headset and switches A2DP-only devices to HFP so the **mic**
appears (the #1 "my BT headset has no microphone" cause).

> **Run it where the human is.** The agent needs only **Bluetooth range to the headset** *and* **network
> reach to the fabric** — it does **not** have to run on the robot's control host. If the headset is out
> of range of that host (the robot is in another room), run this on a **laptop near the human**; it joins
> the same fabric and nothing else in the loop changes.

## Built on Arm Device Connect

The runtime is **Arm's open-source [device-connect](https://github.com/arm/device-connect)**
(`device-connect-edge`, pinned **0.2.4**) — `DeviceRuntime`, the `DeviceDriver` base, `@rpc`/`@emit`,
identity/status types, and the NATS/JWT registration come from there. `HumanAgentDriver` only adds the
headset capture, ASR, grounding, and the `ask`/`notify`/`presence` surface. The contract is **owned
upstream**; the single source of truth for the wrapper is [`lib/README.md`](../lib/README.md).
