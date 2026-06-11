---
name: unitree-g1-voice
description: >-
  Make the Unitree G1 EDU speak to a human and listen for the spoken reply — text-to-speech out
  through the chest speaker (Unitree AudioClient over DDS) and microphone-in via a PulseAudio
  source + pluggable ASR (whisper/vosk/cloud), with the answer grounded to a small set of expected
  choices. Use when the robot needs to ASK A HUMAN FOR HELP and act on the answer, narrate status,
  or characterize the robot's audio I/O for a descriptor. The speaker is fully SDK-supported; the
  mic is NOT exposed by the SDK (capture it at the OS level).
metadata:
  tags: [unitree-g1, audio, voice, tts, asr, speech, human-robot, help-seeking, deviceconnect]
---

# Unitree G1 — Voice (speak + listen)

Speak a question through the chest speaker and listen for the human's answer, then ground it to a
clean decision. The API, the mic workaround, the format rules, and the firmware/VUI gotchas are in
**[`README.md`](README.md)** — this skill is the agent entry point. Module: [`g1_voice.py`](g1_voice.py).

## When to use

- The robot is **stuck and needs help** — ask a person and act on the reply:
  `VoiceIO.ask("Can you hold the far corner?", choices="yesno") → res.choice`.
- **Narrate** status to a human: `VoiceIO.announce("I'm pulling the sheet up now.")`.
- Characterize **audio I/O** for `discover-robot` (speaker = supported; mic = OS-level capture).

## The one thing to know

**Speaking is fully supported; listening is not in the SDK.** Output goes through Unitree's
`AudioClient` (`TtsMaker` / `PlayStream`) over the DDS `"voice"` service. Input is NOT exposed —
capture the mic-array as a **PulseAudio source** (`pactl`) and run ASR yourself (whisper on the
Orin). There is no echo cancellation, so the robot must **speak fully before it listens** (the
module does this for you).

## Minimal use

```python
from g1_voice import VoiceIO
vio = VoiceIO(iface="eth0")                       # off-robot: omit iface (console + keyboard fallback)
res = vio.ask("Should I keep pulling, or are you holding it?",
              choices={"pull": ["pull", "keep going"], "holding": ["holding", "got it"]})
# res.choice in {"pull","holding",None};  res.confidence, res.transcript, res.heard
```

## Try it (on the robot)

```bash
bash install_voice.sh                 # optional: local ASR (faster-whisper) into the env
python _diag_voice.py --iface eth0    # say a test phrase + list mic sources + record/transcribe
```

## Real-to-sim / how it fits the bed-making demo

The behaviour layer fires `ask()` when a **failure predictor** says the robot can't finish on its
own (out of reach / high grip resistance / balance at risk — cf. BCVA, FAIL-Detect), and grounds
the spoken reply (KnowNo-style MCQA) into the next action. Device Connect orchestrates the dialogue
and the shared goal state; the G1's speaker + mic are the human channel.

## Gotchas

- **No echo cancellation** — never record while talking (handled by `say(wait=True)` then record).
- **Built-in VUI assistant** can hold the mic + the `"voice"` service — stop it first.
- **Firmware-gated** — the 4-mic array + voice need recent EDU firmware (~v3.2+).
