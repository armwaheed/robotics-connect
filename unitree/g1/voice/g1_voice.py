#!/usr/bin/env python3
"""
G1 Voice — speak to a human and listen for the reply, on the Unitree G1 EDU.

This is the audio capability the bed-making robot uses to ASK A HUMAN PARTNER FOR HELP when it
gets stuck, and to HEAR the answer (armwaheed/robots#3). It has two halves, because the Unitree
SDK supports them very differently (verified against the SDK source — see README.md):

  SPEAK  (output)  — fully supported. The G1 audio service is a DDS request/response service
                     named "voice"; we drive it through `unitree_sdk2py`'s `AudioClient`:
                     TtsMaker (built-in TTS), PlayStream (raw 16 kHz/mono/16-bit PCM — play your
                     OWN TTS voice), SetVolume, LedControl (chest RGB — used here as an
                     "I'm listening" cue). This half works on the robot out of the box.

  LISTEN (input)   — NOT exposed by the SDK. The `AudioClient` registers an ASR api id (1002) but
                     never calls it; there is no mic-read method (unitree_sdk2_python issue #80).
                     The mic-array *is* reachable as a normal PulseAudio source on the Jetson, so
                     we capture it at the OS level (`parec`) and run ASR ourselves — the same path
                     the OpenMind OM1 G1 stack uses. ASR is pluggable (whisper / vosk / cloud /
                     manual). There is NO acoustic echo cancellation on the input, so we GATE the
                     mic OFF while the robot is talking (speak fully, then listen).

The "ask when stuck → hear the answer → ground it to a decision" loop lives in `VoiceIO.ask`,
which grounds free speech to a small set of expected answers (a KnowNo-style multiple-choice
grounding, arXiv:2307.01928) so the behaviour layer gets a clean decision, not a raw string.

Everything degrades gracefully off-robot: with no SDK the speaker prints to the console, and with
no ASR backend the listener falls back to keyboard input — so the dialog logic is fully testable
in loopback before it ever touches hardware.

GOTCHAS (see README.md for detail)
  * The G1's built-in voice assistant / VUI can HOLD the mic and the "voice" service. Stop it
    before driving audio yourself.
  * Audio is firmware-gated (the 4-mic array + LLM/voice need recent EDU firmware, ~v3.2+).
  * PlayStream PCM must be 16 kHz, mono, 16-bit little-endian (the SDK validator rejects anything
    else). TtsMaker takes plain text and needs no format handling.
"""

from __future__ import annotations

import math
import shutil
import struct
import subprocess
import time
import wave
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Union

# ── DDS channel-factory init (shared across robotics-connect modules) ──────────────────────────
_DDS_INITED = False


def ensure_channel_factory(iface: Optional[str] = None, domain: int = 0) -> None:
    """Initialise the Unitree DDS ChannelFactory exactly once per process.

    `iface` is the network interface that reaches the robot's DDS bus (e.g. "eth0" on the robot,
    or the WiFi/route interface off-board). If another robotics-connect module already initialised
    the factory in this process, call with the same iface — this is a no-op after the first call.
    """
    global _DDS_INITED
    if _DDS_INITED:
        return
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize  # lazy: only on the robot

    if iface:
        ChannelFactoryInitialize(domain, iface)
    else:
        ChannelFactoryInitialize(domain)
    _DDS_INITED = True


# ══════════════════════════════════════════════════════════════════════════════════════════════
#  SPEAK — Unitree AudioClient (TTS + raw PCM + volume + chest LED)
# ══════════════════════════════════════════════════════════════════════════════════════════════

# PlayStream's hard format constraint (the SDK's wav.py validator enforces exactly this).
PCM_RATE = 16000
PCM_CHANNELS = 1
PCM_SAMPLE_WIDTH = 2  # bytes (16-bit)
_PLAYSTREAM_CHUNK = 96000  # bytes per PlayStream call (~3 s @ 16 kHz mono 16-bit), per the SDK example

# Chest-LED cues (R, G, B). Used so a human can SEE the robot's state from across the room.
LED_LISTENING = (0, 80, 160)   # cyan-blue: "I'm listening for your answer"
LED_THINKING = (160, 120, 0)   # amber: "working"
LED_OK = (0, 140, 0)           # green: "got it / done"
LED_OFF = (0, 0, 0)


class G1Speaker:
    """Text-to-speech + audio playback through the G1's speaker, via the DDS `AudioClient`.

    Off-robot (no `unitree_sdk2py`) it prints what it would say, so callers work in loopback.
    """

    def __init__(
        self,
        iface: Optional[str] = None,
        domain: int = 0,
        default_volume: int = 85,
        app_name: str = "robotics-connect",
        init_dds: bool = True,
        wpm: float = 165.0,
        speaker_id: int = 4,
    ) -> None:
        self.iface = iface
        self.domain = domain
        self.default_volume = default_volume
        self.app_name = app_name
        self.wpm = wpm  # words/min, to estimate how long TtsMaker will speak (it returns instantly)
        self._init_dds = init_dds
        # TtsMaker voice. On the tested G1 EDU firmware: 0 = Chinese (female); 1-4 = English (a
        # faint Chinese accent). 4 is the default English voice; override per robot/firmware.
        self.speaker_id = speaker_id
        self._client = None
        self._available = False
        self._tried = False

    # -- lifecycle ------------------------------------------------------------------------------
    def _ensure(self) -> bool:
        if self._tried:
            return self._available
        self._tried = True
        try:
            if self._init_dds:
                ensure_channel_factory(self.iface, self.domain)
            from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

            self._client = AudioClient()
            self._client.SetTimeout(10.0)
            self._client.Init()
            if self.default_volume is not None:
                self._client.SetVolume(int(self.default_volume))
            self._available = True
        except Exception as exc:  # no SDK / no robot / service busy → console fallback
            print(f"[g1-voice] speaker unavailable ({exc!r}); using console fallback.")
            self._available = False
        return self._available

    @property
    def available(self) -> bool:
        return self._ensure()

    # -- speaking -------------------------------------------------------------------------------
    def say(self, text: str, speaker_id: Optional[int] = None, wait: bool = True) -> bool:
        """Speak `text` with the built-in TTS. `TtsMaker` returns immediately, so when `wait` is
        True we block for an estimate of the spoken duration (so the caller can listen *after* the
        robot finishes — there is no echo cancellation). Returns True if it actually spoke.
        `speaker_id` defaults to the constructor's voice (English on the tested firmware)."""
        sid = self.speaker_id if speaker_id is None else speaker_id
        print(f'[g1-voice] 🔊 "{text}"')
        spoke = False
        if self._ensure():
            try:
                code = self._client.TtsMaker(text, sid)
                spoke = (code == 0)
                if code != 0:
                    print(f"[g1-voice] TtsMaker returned code {code}")
            except Exception as exc:
                print(f"[g1-voice] TtsMaker failed: {exc!r}")
        if wait:
            time.sleep(self.estimate_seconds(text))
        return spoke

    def estimate_seconds(self, text: str, floor: float = 1.2) -> float:
        """Estimate how long the TTS will speak `text` (words / wpm), with a small floor + tail."""
        words = max(1, len(text.split()))
        return max(floor, words / max(1.0, self.wpm) * 60.0 + 0.6)

    def play_pcm(self, pcm: bytes, stream_id: Optional[str] = None) -> bool:
        """Play raw 16 kHz / mono / 16-bit-LE PCM through the speaker (e.g. from your own TTS).
        Chunks into PlayStream calls as the SDK example does. Returns True on success."""
        if not self._ensure():
            print(f"[g1-voice] (would play {len(pcm)} PCM bytes)")
            return False
        sid = stream_id or f"{self.app_name}-{int(time.time() * 1000)}"
        try:
            for i in range(0, len(pcm), _PLAYSTREAM_CHUNK):
                self._client.PlayStream(self.app_name, sid, pcm[i:i + _PLAYSTREAM_CHUNK])
                time.sleep(_PLAYSTREAM_CHUNK / (PCM_RATE * PCM_CHANNELS * PCM_SAMPLE_WIDTH))
            return True
        except Exception as exc:
            print(f"[g1-voice] PlayStream failed: {exc!r}")
            return False

    def play_wav(self, path: str) -> bool:
        """Play a 16 kHz / mono / 16-bit WAV file through the speaker."""
        with wave.open(path, "rb") as w:
            if (w.getframerate(), w.getnchannels(), w.getsampwidth()) != (PCM_RATE, PCM_CHANNELS, PCM_SAMPLE_WIDTH):
                raise ValueError(
                    f"{path} must be {PCM_RATE} Hz / {PCM_CHANNELS} ch / 16-bit "
                    f"(got {w.getframerate()} Hz / {w.getnchannels()} ch / {w.getsampwidth() * 8}-bit)"
                )
            return self.play_pcm(w.readframes(w.getnframes()))

    def stop(self) -> None:
        if self._ensure():
            try:
                self._client.PlayStop(self.app_name)
            except Exception:
                pass

    # -- volume + LED ---------------------------------------------------------------------------
    def set_volume(self, volume: int) -> None:
        if self._ensure():
            try:
                self._client.SetVolume(int(volume))
            except Exception as exc:
                print(f"[g1-voice] SetVolume failed: {exc!r}")

    def led(self, rgb: Sequence[int]) -> None:
        """Drive the chest RGB LED (a human-visible state cue). No-op off-robot."""
        if self._ensure():
            try:
                r, g, b = (int(c) for c in rgb)
                self._client.LedControl(r, g, b)
            except Exception:
                pass


# ══════════════════════════════════════════════════════════════════════════════════════════════
#  LISTEN — PulseAudio mic capture (the SDK does not expose the mic) + pluggable ASR
# ══════════════════════════════════════════════════════════════════════════════════════════════

def list_pulse_sources() -> List[str]:
    """`pactl list sources short` — the mic-array shows up here as a PulseAudio source on the
    Jetson (plain ALSA on the Tegra only exposes APE/XBAR virtual devices)."""
    if not shutil.which("pactl"):
        return []
    try:
        out = subprocess.run(["pactl", "list", "sources", "short"], capture_output=True, text=True, timeout=5)
        return [ln for ln in out.stdout.splitlines() if ln.strip()]
    except Exception:
        return []


def _rms(pcm: bytes) -> float:
    """RMS amplitude of 16-bit-LE PCM, normalised to 0..1 — a cheap energy VAD signal."""
    n = len(pcm) // 2
    if n == 0:
        return 0.0
    vals = struct.unpack("<%dh" % n, pcm[: n * 2])
    return math.sqrt(sum(v * v for v in vals) / n) / 32768.0


class G1Microphone:
    """Capture the G1 mic as a PulseAudio source via `parec` (no extra Python dependency).

    `source` is a PulseAudio source name (see `list_pulse_sources`); None uses the default source
    (set it with `pactl set-default-source <name>`, or plug in a USB mic). Off-robot (no `parec`)
    `record` returns b"" so callers fall back to manual input."""

    def __init__(self, source: Optional[str] = None, rate: int = PCM_RATE, channels: int = PCM_CHANNELS) -> None:
        self.source = source
        self.rate = rate
        self.channels = channels

    @property
    def available(self) -> bool:
        return shutil.which("parec") is not None

    def _parec_cmd(self) -> List[str]:
        cmd = ["parec", "--format=s16le", f"--rate={self.rate}", f"--channels={self.channels}", "--raw"]
        if self.source:
            cmd.append(f"--device={self.source}")
        return cmd

    def record(self, seconds: float) -> bytes:
        """Record a fixed window of 16-bit-LE PCM. Simple and robust; pair with a timeout."""
        if not self.available:
            return b""
        nbytes = int(seconds * self.rate * self.channels * PCM_SAMPLE_WIDTH)
        try:
            proc = subprocess.run(self._parec_cmd(), capture_output=True, timeout=seconds + 3.0,
                                  input=None, stdin=subprocess.DEVNULL)
            return proc.stdout[:nbytes]
        except subprocess.TimeoutExpired as e:
            return (e.stdout or b"")[:nbytes]
        except Exception:
            return b""

    def record_utterance(
        self,
        max_seconds: float = 6.0,
        silence_rms: float = 0.012,
        silence_hold: float = 0.8,
        start_timeout: float = 3.0,
        chunk: float = 0.2,
    ) -> bytes:
        """Record until the speaker goes quiet (energy VAD), or `max_seconds`. Streams `parec` in
        `chunk`-second reads: waits up to `start_timeout` for speech to begin, then stops after
        `silence_hold` s below `silence_rms`. Falls back to a fixed window if streaming is
        unavailable."""
        if not self.available:
            return b""
        frame_bytes = int(chunk * self.rate * self.channels * PCM_SAMPLE_WIDTH)
        proc = None
        try:
            proc = subprocess.Popen(self._parec_cmd(), stdout=subprocess.PIPE, stdin=subprocess.DEVNULL)
            collected = bytearray()
            started = False
            t0 = time.time()
            last_voice = t0
            while time.time() - t0 < max_seconds:
                buf = proc.stdout.read(frame_bytes)
                if not buf:
                    break
                level = _rms(buf)
                if level >= silence_rms:
                    started = True
                    last_voice = time.time()
                    collected += buf
                elif started:
                    collected += buf
                    if time.time() - last_voice >= silence_hold:
                        break
                elif time.time() - t0 >= start_timeout:
                    break  # nobody spoke
            return bytes(collected)
        except Exception:
            return self.record(max_seconds)
        finally:
            if proc is not None:
                try:
                    proc.terminate()
                    proc.wait(timeout=1.0)
                except Exception:
                    pass


# -- ASR backends -------------------------------------------------------------------------------
ASRFn = Callable[[bytes, int], str]


def make_asr(backend: str = "auto", model: Optional[str] = None) -> ASRFn:
    """Return a transcribe(pcm, rate)->str function.

    backend: "whisper" (faster-whisper or openai-whisper, local on the Orin — recommended),
             "vosk" (lighter, streaming), "manual" (keyboard — for loopback / off-robot),
             "auto" (whisper → vosk → manual, first that imports).
    """
    order = {
        "auto": ["whisper", "vosk", "manual"],
        "whisper": ["whisper"],
        "vosk": ["vosk"],
        "manual": ["manual"],
    }.get(backend, [backend])

    for name in order:
        try:
            if name == "whisper":
                return _whisper_asr(model or "base.en")
            if name == "vosk":
                return _vosk_asr(model)
            if name == "manual":
                return _manual_asr()
        except Exception as exc:
            print(f"[g1-voice] ASR backend '{name}' unavailable ({exc!r})")
            continue
    return _manual_asr()


def _whisper_asr(model_name: str) -> ASRFn:
    try:
        from faster_whisper import WhisperModel  # CUDA on the Orin if available

        model = WhisperModel(model_name, device="auto", compute_type="int8")

        def transcribe(pcm: bytes, rate: int) -> str:
            import numpy as np

            audio = np.frombuffer(pcm, dtype=np.int16).astype(np.float32) / 32768.0
            segs, _ = model.transcribe(audio, language="en", sampling_rate=rate)
            return " ".join(s.text for s in segs).strip()

        return transcribe
    except Exception:
        import whisper  # openai-whisper fallback

        model = whisper.load_model(model_name.replace(".en", ""))

        def transcribe(pcm: bytes, rate: int) -> str:
            import numpy as np

            audio = np.frombuffer(pcm, dtype=np.int16).astype(np.float32) / 32768.0
            return model.transcribe(audio, language="en", fp16=False)["text"].strip()

        return transcribe


def _vosk_asr(model_path: Optional[str]) -> ASRFn:
    import json as _json

    from vosk import KaldiRecognizer, Model

    model = Model(model_path) if model_path else Model(lang="en-us")

    def transcribe(pcm: bytes, rate: int) -> str:
        rec = KaldiRecognizer(model, rate)
        rec.AcceptWaveform(pcm)
        return _json.loads(rec.FinalResult()).get("text", "").strip()

    return transcribe


def _manual_asr() -> ASRFn:
    def transcribe(pcm: bytes, rate: int) -> str:
        try:
            return input("[g1-voice] (manual ASR) type the human's reply > ").strip()
        except EOFError:
            return ""

    return transcribe


# ══════════════════════════════════════════════════════════════════════════════════════════════
#  ASK — speak a question, listen, and ground the reply to a decision (the help-seeking loop)
# ══════════════════════════════════════════════════════════════════════════════════════════════

@dataclass
class AskResult:
    question: str
    transcript: str = ""
    choice: Optional[str] = None      # the grounded decision (a key of `choices`), or None
    confidence: float = 0.0           # 0..1, fraction of the matched option's keywords hit
    heard: bool = False               # did we capture any speech at all?
    options: Dict[str, List[str]] = field(default_factory=dict)


# Default yes/no grounding vocabulary (the most common help-ask).
YES_NO = {
    "yes": ["yes", "yeah", "yep", "yup", "sure", "ok", "okay", "go", "go ahead", "do it", "affirmative", "i can", "got it", "done"],
    "no": ["no", "nope", "not", "don't", "do not", "stop", "wait", "hold on", "negative", "can't", "cannot"],
}


def ground(transcript: str, choices: Dict[str, List[str]]) -> tuple[Optional[str], float]:
    """Ground free speech to one of `choices` (label → keyword list) by keyword overlap — a simple,
    on-device version of KnowNo's multiple-choice grounding. Returns (label, confidence). If nothing
    matches, (None, 0.0); the caller can then re-ask (KnowNo: ask again when >1 option survives)."""
    t = " " + transcript.lower().strip() + " "
    best_label, best_score = None, 0.0
    for label, kws in choices.items():
        hits = sum(1 for kw in kws if (" " + kw.lower() + " ") in t or kw.lower() in t.split())
        if hits:
            score = hits / max(1, len(kws)) + 0.001 * hits  # tie-break toward more hits
            if score > best_score:
                best_label, best_score = label, min(1.0, score)
    return best_label, best_score


class VoiceIO:
    """The robot's voice channel to its human partner: announce, and ask-then-listen-then-ground.

    Holds a `G1Speaker` and a `G1Microphone` + ASR; on the robot these are real, off-robot they
    fall back to console + keyboard so the help-seeking dialog is testable in loopback.
    """

    def __init__(
        self,
        speaker: Optional[G1Speaker] = None,
        mic: Optional[G1Microphone] = None,
        asr: Optional[ASRFn] = None,
        iface: Optional[str] = None,
        asr_backend: str = "auto",
    ) -> None:
        self.speaker = speaker or G1Speaker(iface=iface)
        self.mic = mic or G1Microphone()
        self.asr = asr or make_asr(asr_backend)

    def announce(self, text: str) -> None:
        """Say something that needs no reply (status narration to the human)."""
        self.speaker.led(LED_THINKING)
        self.speaker.say(text, wait=True)
        self.speaker.led(LED_OFF)

    def ask(
        self,
        question: str,
        choices: Optional[Union[Dict[str, List[str]], Sequence[str]]] = "yesno",
        listen_seconds: float = 6.0,
        retries: int = 1,
    ) -> AskResult:
        """Speak `question`, listen for the human's spoken reply, and ground it to one of `choices`.

        `choices`: "yesno" (default) → yes/no; a dict {label: [keywords]}; a list of labels (each
        label is its own keyword); or None (return the raw transcript, no grounding). On an unclear
        reply, re-ask up to `retries` times (KnowNo: keep asking until the answer disambiguates).

        The mic is GATED while the robot talks (no echo cancellation): `say(wait=True)` blocks for
        the spoken duration, and only then do we record.
        """
        opts = self._resolve_choices(choices)
        result = AskResult(question=question, options=opts or {})
        for attempt in range(retries + 1):
            prompt = question if attempt == 0 else f"Sorry, I didn't catch that. {question}"
            self.speaker.say(prompt, wait=True)            # speak fully (mic gated until done)
            self.speaker.led(LED_LISTENING)                # human-visible "your turn"
            pcm = self.mic.record_utterance(max_seconds=listen_seconds)
            self.speaker.led(LED_OFF)
            transcript = self._transcribe(pcm)
            result.transcript = transcript
            result.heard = bool(transcript)
            if opts is None:
                return result
            label, conf = ground(transcript, opts)
            result.choice, result.confidence = label, conf
            if label is not None:
                self.speaker.led(LED_OK)
                return result
        return result

    # -- helpers --------------------------------------------------------------------------------
    def _transcribe(self, pcm: bytes) -> str:
        if not pcm and not self.mic.available:
            # off-robot / no mic: ask the ASR fn (manual backend reads the keyboard)
            return self.asr(b"", self.mic.rate)
        if not pcm:
            return ""
        try:
            return self.asr(pcm, self.mic.rate)
        except Exception as exc:
            print(f"[g1-voice] ASR failed: {exc!r}")
            return ""

    @staticmethod
    def _resolve_choices(
        choices: Optional[Union[Dict[str, List[str]], Sequence[str]]],
    ) -> Optional[Dict[str, List[str]]]:
        if choices is None:
            return None
        if choices == "yesno":
            return dict(YES_NO)
        if isinstance(choices, dict):
            return {k: list(v) for k, v in choices.items()}
        return {label: [label] for label in choices}  # list of labels → each is its own keyword


if __name__ == "__main__":
    # Quick self-check: works off-robot (console speaker + keyboard ASR) and on-robot (real audio).
    import argparse

    ap = argparse.ArgumentParser(description="G1 voice self-check (ask the human a yes/no question).")
    ap.add_argument("--iface", default=None, help="DDS network interface to the robot (e.g. eth0).")
    ap.add_argument("--asr", default="auto", help="ASR backend: auto|whisper|vosk|manual")
    ap.add_argument("--question", default="Can you hold the far corner of the sheet for me?")
    args = ap.parse_args()

    print("PulseAudio sources:", *(["", *list_pulse_sources()] or ["(none / off-robot)"]), sep="\n  ")
    vio = VoiceIO(iface=args.iface, asr_backend=args.asr)
    res = vio.ask(args.question, choices="yesno")
    print(f"\nheard={res.heard!r}  transcript={res.transcript!r}  →  choice={res.choice!r} ({res.confidence:.2f})")
