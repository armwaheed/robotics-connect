# Unitree G1 — Voice (speak to a human, listen for the reply)

The G1's audio channel to a human partner: **speak** a question through the chest speaker and
**listen** for the spoken answer, then ground that answer to a clean decision. Built for
[armwaheed/robots#3](https://github.com/armwaheed/robots/issues/3) — a bed-making G1 that, when it
gets stuck, **asks a person for help** and acts on what it hears.

The module is [`g1_voice.py`](g1_voice.py). It has two halves that the Unitree SDK supports very
differently — read this before you wire it up.

---

## TL;DR

```python
from g1_voice import VoiceIO

vio = VoiceIO(iface="eth0")                      # on the robot; off-robot omit iface
res = vio.ask("Can you hold the far corner of the sheet for me?", choices="yesno")
if res.choice == "yes":
    ...                                          # the human agreed — co-manipulate
```

`ask()` speaks the question, lights the chest LED so the human knows it's their turn, records the
reply, transcribes it, and **grounds** it to one of the expected answers (`res.choice`,
`res.confidence`). On the robot it uses the real speaker + mic; off-robot it falls back to the
console + keyboard, so the whole dialog is testable in loopback.

---

## 1. Speak — fully supported (Unitree `AudioClient` over DDS)

The G1 audio service is a DDS request/response service named **`"voice"`**, driven through
`unitree_sdk2py`'s `AudioClient`. Verified methods (from the SDK source):

| Method | What it does |
|---|---|
| `TtsMaker(text, speaker_id=0)` | built-in text-to-speech (English works; `speaker_id=0` is the default voice) |
| `PlayStream(app, stream_id, pcm)` | play **raw PCM** — your OWN TTS voice (Piper/ElevenLabs/…), chunked |
| `SetVolume(0..100)` / `GetVolume()` | speaker volume |
| `LedControl(R,G,B)` | chest RGB LED — used here as a human-visible **"I'm listening"** cue |
| `PlayStop(app)` | stop a stream |

`G1Speaker` wraps these: `say(text)`, `play_pcm(bytes)`, `play_wav(path)`, `set_volume(v)`,
`led(rgb)`, `stop()`. `TtsMaker` returns immediately, so `say(..., wait=True)` blocks for an
estimate of the spoken duration — this is also how we **gate the mic** (see §2).

**PlayStream format is hard-constrained: 16 kHz, mono, 16-bit little-endian PCM** (the SDK
validator rejects anything else). Resample your own TTS to that before `play_pcm`. `TtsMaker`
takes plain text and needs no format handling.

## 2. Listen — NOT in the SDK; capture the mic at the OS level

**The Unitree SDK does not expose the microphone.** `AudioClient` registers an ASR api id (1002)
but never calls it; there is no mic-read method (confirmed in
[unitree_sdk2_python#80](https://github.com/unitreerobotics/unitree_sdk2_python/issues/80)). The
mic-array audio is published on the undocumented DDS topic `rt/audiosender`
(`unitree_go::msg::dds_::AudioData_`, no format metadata) — fragile, so we don't rely on it.

The reliable path (the one [OpenMind OM1](https://github.com/OpenMind/OM1) uses) is to capture the
mic as a normal **PulseAudio source** on the Jetson and run ASR yourself. Plain ALSA on the Tegra
only shows APE/XBAR virtual devices, so go through PulseAudio:

```bash
pactl list sources short          # find the mic-array source
pactl set-default-source <name>   # select it (or plug in a USB mic and select that)
```

`G1Microphone` captures via `parec` (no extra Python dependency): `record(seconds)` for a fixed
window, or `record_utterance()` for energy-VAD capture (waits for speech, stops on silence).

**ASR is pluggable** (`make_asr(backend=...)`):

| backend | use |
|---|---|
| `whisper` | local on the Orin — `faster-whisper` (CUDA, recommended) or `openai-whisper` |
| `vosk` | lighter / streaming / wake-word-friendly |
| `manual` | keyboard input — loopback / off-robot testing |
| `auto` | first of whisper → vosk → manual that imports (default) |

### On-robot finding (G1 EDU, verified 2026-06): the native array is a *closed* capture

On a live G1 EDU we exhausted every native path to the 4-mic array and confirmed it is **not
readable from userspace**: `AudioClient` doesn't expose it (ASR api `1002` is registered but returns
`3104`); the only ALSA capture node `pcmC1D0c` is held **exclusively** by the factory; even with
"Wake-up Conversation Mode" on, the wake word opens the mic but the route is **not on the XBAR mux**
(`ADMAIF1 Mux = None`) so it can't be fanned to a parallel ADMAIF (every DMIC/I2S tap reads
noise/silence); nothing is republished on `rt/audiosender`/`rt/audio_msg`; and there is **no
on-board ASR process or voice log** (`master_service` only supervises `ota_pipe` + `video_hub_pc4`).
**Conclusion:** wake-up mode streams the mic *off-robot* to Unitree's app/cloud — there is no local
hook. So for "the robot listens," prefer a **USB mic** (clean PulseAudio source — `record_utterance`
works unchanged) or route the human in as a **Device Connect agent** (a headset + DC sidecar on the
control host). Tracking the supported on-board path with Unitree in
[robotics-connect#1](https://github.com/armwaheed/robotics-connect/issues/1).

## 3. Ask → listen → ground (the help-seeking loop)

`VoiceIO.ask(question, choices)` is the loop the behaviour layer calls. `choices` is `"yesno"`
(default), a `{label: [keywords]}` dict, a list of labels, or `None` (raw transcript). It returns
an `AskResult(question, transcript, choice, confidence, heard)`.

Grounding maps free speech to one of the choices by keyword overlap — a simple, on-device version
of **KnowNo**'s multiple-choice grounding ([arXiv:2307.01928](https://arxiv.org/abs/2307.01928)):
the human can say "yeah, go ahead" and the robot reads `choice="yes"`. On an unclear reply it
re-asks (KnowNo: keep asking until the answer disambiguates).

```python
res = vio.ask("Should I keep pulling, or are you holding it?",
              choices={"pull": ["pull", "keep going", "yes"],
                       "holding": ["holding", "got it", "i have it"],
                       "wait": ["wait", "stop", "hold on"]})
```

---

## Gotchas (the ones that will bite you)

- **No acoustic echo cancellation on input.** Don't record while the robot is talking or it hears
  itself. `ask()` speaks fully (`say(wait=True)`) and only *then* records — keep that ordering.
- **The built-in voice assistant / VUI can hold the mic and the `"voice"` service.** If the
  factory "Hello robot" assistant is running, your capture and/or TTS may be contended — stop it
  before driving audio yourself.
- **Audio is firmware-gated.** The 4-mic array + LLM/voice features need recent EDU firmware
  (~v3.2+). Verify your firmware and the `voice` service state first.
- **DDS init is once-per-process.** `ensure_channel_factory(iface)` is idempotent; if another
  robotics-connect module already initialised DDS in your process, pass `init_dds=False` to
  `G1Speaker` (or just call once with the same `iface`).

## Install (on the robot)

```bash
bash install_voice.sh              # installs faster-whisper into the robotics-connect env (optional ASR)
python _diag_voice.py --iface eth0 # speak a test phrase, list mic sources, record + transcribe
```

The speaker works with no extra install (just `unitree_sdk2py`, already on the robot). ASR needs a
backend (`faster-whisper` recommended on the Orin); without one the listener falls back to manual.

## What it feeds `discover-robot`

| Descriptor field | Value |
|---|---|
| `audio.speaker` | Unitree chest speaker via `AudioClient` (`"voice"` DDS service) — TTS + raw PCM |
| `audio.microphone` | 4-mic array, **not** in the SDK; capture via PulseAudio source + local ASR |
| `audio.tts` | built-in `TtsMaker` (+ bring-your-own via `PlayStream`, 16 kHz/mono/16-bit) |
| `audio.asr` | bring-your-own (whisper/vosk/cloud) over a PulseAudio-captured mic stream |

## Sources

SDK source the API was read from (signatures verbatim): `unitree_sdk2py/g1/audio/g1_audio_client.py`,
`g1_audio_api.py`, `example/g1/audio/{g1_audio_client_example.py,wav.py}`;
[issue #80](https://github.com/unitreerobotics/unitree_sdk2_python/issues/80) (mic missing);
OM1 G1 docs (PulseAudio mic + ASR). Official: support.unitree.com `G1_developer/VuiClient_Service`.
