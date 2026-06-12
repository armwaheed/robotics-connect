#!/usr/bin/env python3
"""Robust Bluetooth-headset audio I/O — works with Bluetooth headsets in general.

This is the audio layer behind the **human-as-a-Device-Connect-agent**: a person on the
DGX Spark wears *any* Bluetooth headset, the robot ASKS a question (played into the
earpiece) and the person's spoken answer is captured from the headset mic, transcribed,
and grounded to a decision. It is deliberately not tied to one headset model.

What "robust to Bluetooth headsets in general" means here:

  * **Auto-detect a connected BT headset** across PipeWire / PulseAudio / ALSA, keying off
    the LIVE source/sink nodes — never the cached ``bluez5.profile`` device prop, which goes
    stale (a headset can report ``profile=off`` while its sink/source are plainly active).
  * **Fix the "Bluetooth headset has no microphone" case.** Most headsets connect in the
    A2DP profile (high-quality output, NO mic). To get the mic you must put the card in the
    HFP ``headset-head-unit`` profile. If we see a BT card with a sink but no source, we
    switch it to the best available ``headset-head-unit*`` profile (mSBC > CVSD > bare) and
    re-resolve. This is the single most common reason a BT mic "doesn't work".
  * **Degrade gracefully** to the system default source/sink (USB headset, built-in mic)
    and finally to the backend default, so the module still runs with no Bluetooth at all.
  * **Override anything** via env (``HUMAN_AGENT_SOURCE`` / ``HUMAN_AGENT_SINK`` /
    ``HUMAN_AGENT_BT_MAC`` / ``HUMAN_AGENT_AUDIO_BACKEND``) or constructor args.

Capture is 16 kHz / mono / 16-bit-LE — what both Whisper and HFP want. The sibling module
``unitree/g1/voice/g1_voice.py`` does the same job for the *robot's own* chest speaker + mic
(Unitree ``AudioClient`` over DDS); this one is the *human's* headset on the compute node.
"""

from __future__ import annotations

import json
import math
import os
import shutil
import struct
import subprocess
import tempfile
import time
import wave
from dataclasses import dataclass
from typing import List, Optional, Tuple

# 16 kHz / mono / 16-bit-LE — Whisper's native rate and the HFP wideband (mSBC) rate.
RATE = 16000
CHANNELS = 1
WIDTH = 2  # bytes per sample (16-bit)

# Bluez profile names that expose a MICROPHONE (HFP/HSP "head unit"), best codec first.
# A2DP profiles (``a2dp-sink*``) are deliberately NOT here — they have no mic.
_HFP_PREFERENCE = ("headset-head-unit-msbc", "headset-head-unit-cvsd", "headset-head-unit")


def _which(*names: str) -> Optional[str]:
    for n in names:
        if shutil.which(n):
            return n
    return None


# ══════════════════════════════════════════════════════════════════════════════════════════════
#  Endpoint discovery
# ══════════════════════════════════════════════════════════════════════════════════════════════

@dataclass
class Endpoint:
    """A resolved capture+playback pair and how we got there."""

    backend: str                      # "pipewire" | "pulseaudio" | "alsa"
    source: Optional[str] = None      # capture node/device name (None = backend default)
    sink: Optional[str] = None        # playback node/device name (None = backend default)
    is_bluetooth: bool = False
    bt_name: Optional[str] = None
    bt_mac: Optional[str] = None
    bt_card_id: Optional[int] = None  # PipeWire device id (for profile switching)
    profile: Optional[str] = None     # active card profile, if known
    note: str = ""                    # human-readable trail of how this was resolved

    def describe(self) -> str:
        who = self.bt_name or (self.source or "default source")
        kind = "Bluetooth headset" if self.is_bluetooth else "audio device"
        prof = f", profile={self.profile}" if self.profile else ""
        return f"{kind} '{who}' via {self.backend}{prof} (mic={self.source or 'default'}, out={self.sink or 'default'})"


def _pick_backend(override: Optional[str] = None) -> str:
    override = override or os.environ.get("HUMAN_AGENT_AUDIO_BACKEND")
    if override:
        return override
    if _which("pw-record") and _which("pw-play"):
        return "pipewire"
    if _which("parec") and _which("paplay"):
        return "pulseaudio"
    if _which("arecord") and _which("aplay"):
        return "alsa"
    raise RuntimeError(
        "No audio CLI found. Install PipeWire (pw-record/pw-play), PulseAudio "
        "(parec/paplay), or ALSA (arecord/aplay)."
    )


def _pw_dump() -> Optional[list]:
    """Full PipeWire object graph as a list of dicts (``pw-dump``), or None if unavailable."""
    if not _which("pw-dump"):
        return None
    try:
        out = subprocess.run(["pw-dump"], capture_output=True, text=True, timeout=8)
        return json.loads(out.stdout) if out.stdout.strip() else None
    except Exception:
        return None


def _pw_bt_cards(objs: list) -> List[dict]:
    """Bluez cards in the graph, each annotated with its live nodes and available profiles."""
    cards: List[dict] = []
    for o in objs:
        if o.get("type") != "PipeWire:Interface:Device":
            continue
        props = (o.get("info") or {}).get("props") or {}
        if props.get("device.api") != "bluez5":
            continue
        params = (o.get("info") or {}).get("params") or {}
        active = params.get("Profile") or []
        cards.append({
            "id": o.get("id"),
            "name": props.get("device.name"),
            "desc": props.get("device.description") or props.get("api.bluez5.icon"),
            "mac": props.get("api.bluez5.address"),
            "active_profile": (active[0].get("name") if active else None),
            "enum_profiles": [
                {"index": p.get("index"), "name": p.get("name"), "available": p.get("available")}
                for p in (params.get("EnumProfile") or [])
            ],
        })
    return cards


def _pw_nodes_for_mac(objs: list, mac: str) -> Tuple[Optional[str], Optional[str]]:
    """(source_node_name, sink_node_name) for the bluez device with this MAC, or (None, None)."""
    source = sink = None
    for o in objs:
        if o.get("type") != "PipeWire:Interface:Node":
            continue
        props = (o.get("info") or {}).get("props") or {}
        if props.get("api.bluez5.address") != mac:
            continue
        cls = props.get("media.class")
        name = props.get("node.name")
        if cls == "Audio/Source":
            source = name
        elif cls == "Audio/Sink":
            sink = name
    return source, sink


def _pw_set_profile(card_id: int, profile_index: int) -> bool:
    """Switch a bluez card to a profile index via ``wpctl set-profile``. Returns success."""
    if not _which("wpctl"):
        return False
    try:
        r = subprocess.run(["wpctl", "set-profile", str(card_id), str(profile_index)],
                           capture_output=True, text=True, timeout=6)
        return r.returncode == 0
    except Exception:
        return False


def _ensure_hfp(card: dict) -> dict:
    """If a BT card has no live mic, switch it to the best HFP profile so the mic appears.

    Returns the (possibly refreshed) card dict with live source/sink filled in. No-op when the
    card already has a source, or when no ``headset-head-unit*`` profile is available."""
    objs = _pw_dump() or []
    source, sink = _pw_nodes_for_mac(objs, card["mac"]) if card.get("mac") else (None, None)
    card["source"], card["sink"] = source, sink
    if source:  # mic already live — nothing to do
        return card

    # Find the best available HFP profile and switch to it.
    by_name = {p["name"]: p for p in card.get("enum_profiles", []) if p.get("name")}
    target = next((by_name[n] for n in _HFP_PREFERENCE
                   if n in by_name and by_name[n].get("available") in ("yes", True, None)), None)
    if not target or card.get("id") is None:
        return card  # can't get a mic on this headset (no HFP) — leave sink-only

    if _pw_set_profile(card["id"], target["index"]):
        for _ in range(10):  # wait for the new source node to appear (~1 s)
            time.sleep(0.15)
            objs = _pw_dump() or []
            source, sink = _pw_nodes_for_mac(objs, card["mac"])
            if source:
                break
        card["source"], card["sink"] = source, sink
        card["active_profile"] = target["name"]
    return card


def discover(prefer_bluetooth: bool = True,
             source_override: Optional[str] = None,
             sink_override: Optional[str] = None,
             bt_mac: Optional[str] = None) -> Endpoint:
    """Resolve the headset endpoint to use.

    Order: explicit overrides → a connected BT headset (switching it to HFP if needed) →
    the system default source/sink. ``bt_mac`` pins a specific headset by MAC."""
    backend = _pick_backend()
    source_override = source_override or os.environ.get("HUMAN_AGENT_SOURCE")
    sink_override = sink_override or os.environ.get("HUMAN_AGENT_SINK")
    bt_mac = bt_mac or os.environ.get("HUMAN_AGENT_BT_MAC")

    if source_override or sink_override:
        return Endpoint(backend=backend, source=source_override, sink=sink_override,
                        note="explicit override")

    if prefer_bluetooth and backend == "pipewire":
        objs = _pw_dump() or []
        cards = _pw_bt_cards(objs)
        if bt_mac:
            cards = [c for c in cards if c.get("mac") == bt_mac]
        # Prefer a card we can give BOTH a mic and a speaker (a true headset).
        best = None
        for card in cards:
            card = _ensure_hfp(card)
            if card.get("source") and card.get("sink"):
                best = card
                break
            if card.get("source") and best is None:
                best = card  # mic-only is still usable
        if best is None and cards:
            best = _ensure_hfp(cards[0])  # sink-only headset (output only)
        if best:
            return Endpoint(
                backend=backend, source=best.get("source"), sink=best.get("sink"),
                is_bluetooth=True, bt_name=best.get("desc") or best.get("name"),
                bt_mac=best.get("mac"), bt_card_id=best.get("id"),
                profile=best.get("active_profile"),
                note="auto-detected Bluetooth headset",
            )

    # Fall back to the backend default source/sink.
    return Endpoint(backend=backend, source=None, sink=None,
                    note="system default source/sink (no Bluetooth headset matched)")


# ══════════════════════════════════════════════════════════════════════════════════════════════
#  Capture + playback + TTS
# ══════════════════════════════════════════════════════════════════════════════════════════════

def _rms(pcm: bytes) -> float:
    n = len(pcm) // 2
    if n == 0:
        return 0.0
    vals = struct.unpack("<%dh" % n, pcm[: n * 2])
    return math.sqrt(sum(v * v for v in vals) / n) / 32768.0


class Headset:
    """Record from / play to a resolved :class:`Endpoint`, across PipeWire / Pulse / ALSA."""

    def __init__(self, endpoint: Optional[Endpoint] = None, tts: Optional["TTS"] = None):
        self.ep = endpoint or discover()
        self.tts = tts if tts is not None else TTS()

    # -- capability ----------------------------------------------------------------------------
    def present(self) -> bool:
        """True if a microphone is currently capturable (headset/mic connected).

        Cheap and non-mutating — safe to call from a status poll on the event loop. It reports the
        endpoint resolved at startup/last refresh; call refresh() explicitly to re-resolve a
        headset that re-connected or changed profile."""
        if self.ep.backend == "pipewire" and self.ep.is_bluetooth:
            return bool(self.ep.source)
        return True  # a default source is assumed present off-Bluetooth

    def refresh(self) -> None:
        """Re-resolve the endpoint (handles a headset that re-connected or changed profile)."""
        try:
            self.ep = discover(prefer_bluetooth=True, bt_mac=self.ep.bt_mac)
        except Exception:
            pass

    def info(self) -> dict:
        ep = self.ep
        return {
            "backend": ep.backend, "bluetooth": ep.is_bluetooth, "headset": ep.bt_name,
            "mac": ep.bt_mac, "profile": ep.profile, "source": ep.source, "sink": ep.sink,
            "tts": self.tts.name, "description": ep.describe(),
        }

    # -- recording (raw stdout streaming with energy VAD) --------------------------------------
    def _rec_cmd(self) -> List[str]:
        be, src = self.ep.backend, self.ep.source
        if be == "pipewire":
            cmd = ["pw-record", "--rate", str(RATE), "--channels", str(CHANNELS),
                   "--format", "s16", "--latency", "50ms"]
            if src:
                cmd += ["--target", src]
            cmd += ["-"]  # raw/WAV to stdout (header auto-detected on read)
            return cmd
        if be == "pulseaudio":
            cmd = ["parec", "--format=s16le", f"--rate={RATE}", f"--channels={CHANNELS}", "--raw"]
            if src:
                cmd += [f"--device={src}"]
            return cmd
        # alsa
        cmd = ["arecord", "-q", "-f", "S16_LE", "-r", str(RATE), "-c", str(CHANNELS), "-t", "raw"]
        if src:
            cmd += ["-D", src]
        return cmd

    @staticmethod
    def _strip_wav_header(first: bytes) -> bytes:
        """pw-record may emit a RIFF/WAV header on stdout; skip it to get raw PCM."""
        if first[:4] == b"RIFF" and b"data" in first[:128]:
            idx = first.find(b"data")
            return first[idx + 8:]  # past 'data' + 4-byte size
        return first

    def record_utterance(self, max_seconds: float = 7.0, silence_rms: float = 0.012,
                         silence_hold: float = 0.9, start_timeout: float = 4.0,
                         chunk: float = 0.2) -> bytes:
        """Stream-record until the speaker goes quiet (energy VAD) or ``max_seconds``.

        Waits up to ``start_timeout`` for speech to begin, then stops ``silence_hold`` s after the
        level drops below ``silence_rms``. Returns 16 kHz/mono/16-bit-LE PCM (b"" if nothing)."""
        frame = int(chunk * RATE * CHANNELS * WIDTH)
        proc = None
        try:
            proc = subprocess.Popen(self._rec_cmd(), stdout=subprocess.PIPE,
                                    stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL)
            collected = bytearray()
            started = False
            header_done = False
            t0 = time.time()
            last_voice = t0
            while time.time() - t0 < max_seconds:
                buf = proc.stdout.read(frame)
                if not buf:
                    break
                if not header_done:
                    buf = self._strip_wav_header(buf)
                    header_done = True
                    if not buf:
                        continue
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
            return b""
        finally:
            if proc is not None:
                try:
                    proc.terminate()
                    proc.wait(timeout=1.0)
                except Exception:
                    try:
                        proc.kill()
                    except Exception:
                        pass

    # -- playback -------------------------------------------------------------------------------
    def play_wav(self, path: str) -> bool:
        be, sink = self.ep.backend, self.ep.sink
        try:
            if be == "pipewire":
                cmd = ["pw-play"] + (["--target", sink] if sink else []) + [path]
            elif be == "pulseaudio":
                cmd = ["paplay"] + ([f"--device={sink}"] if sink else []) + [path]
            else:
                cmd = ["aplay", "-q"] + (["-D", sink] if sink else []) + [path]
            return subprocess.run(cmd, capture_output=True, timeout=60).returncode == 0
        except Exception:
            return False

    def play_tone(self, freq: float = 880.0, secs: float = 0.18, volume: float = 0.5) -> bool:
        """Play a short tone to the earpiece — a 'your turn to speak' earcon before recording.

        In out-loud mode the robot's speaker carries the question; this earcon tells the human
        exactly WHEN the agent starts listening (turn-taking cue), which a one-way speaker can't."""
        n = int(secs * RATE)
        fade = max(1, int(0.01 * RATE))  # 10 ms in/out fade to avoid clicks
        frames = bytearray()
        for i in range(n):
            amp = volume * min(1.0, i / fade, (n - i) / fade)
            frames += struct.pack("<h", int(amp * 32767 * math.sin(2 * math.pi * freq * i / RATE)))
        fd, path = tempfile.mkstemp(suffix=".wav", prefix="ha_cue_")
        os.close(fd)
        try:
            with wave.open(path, "wb") as wf:
                wf.setnchannels(CHANNELS)
                wf.setsampwidth(WIDTH)
                wf.setframerate(RATE)
                wf.writeframes(bytes(frames))
            return self.play_wav(path)
        finally:
            try:
                os.unlink(path)
            except Exception:
                pass

    def say(self, text: str) -> bool:
        """Speak ``text`` into the earpiece via the TTS backend. Best-effort (False if no TTS)."""
        wav = self.tts.to_wav(text)
        if not wav:
            print(f'[human-agent] (no TTS backend; would say) "{text}"')
            return False
        try:
            ok = self.play_wav(wav)
        finally:
            try:
                os.unlink(wav)
            except Exception:
                pass
        return ok


# ══════════════════════════════════════════════════════════════════════════════════════════════
#  Text-to-speech (pluggable, best-effort): piper → espeak-ng → none
# ══════════════════════════════════════════════════════════════════════════════════════════════

def _find_piper_voice() -> Optional[str]:
    env = os.environ.get("HUMAN_AGENT_PIPER_VOICE")
    if env and os.path.exists(env):
        return env
    for root in (os.path.expanduser("~/.local/share/piper-voices"),
                 os.path.expanduser("~/.local/share/piper"), "/usr/share/piper-voices"):
        if not os.path.isdir(root):
            continue
        for dirpath, _dirs, files in os.walk(root):
            for f in sorted(files):
                if f.endswith(".onnx"):
                    return os.path.join(dirpath, f)
    return None


class TTS:
    """Render text to a temp 16-bit WAV. Tries piper (offline neural), then espeak-ng."""

    def __init__(self) -> None:
        self.name = "none"
        self._piper = None
        self._voice_path = _find_piper_voice()
        if self._voice_path:
            try:
                from piper import PiperVoice
                self._piper = PiperVoice.load(self._voice_path)
                self.name = f"piper:{os.path.basename(self._voice_path)}"
                return
            except Exception as exc:
                print(f"[human-agent] piper unavailable ({exc!r})")
        if _which("espeak-ng", "espeak"):
            self.name = "espeak-ng"

    def to_wav(self, text: str) -> Optional[str]:
        if self._piper is not None:
            try:
                fd, path = tempfile.mkstemp(suffix=".wav", prefix="ha_tts_")
                os.close(fd)
                with wave.open(path, "wb") as wf:
                    self._piper.synthesize_wav(text, wf)
                return path
            except Exception as exc:
                print(f"[human-agent] piper synth failed: {exc!r}")
        exe = _which("espeak-ng", "espeak")
        if exe:
            try:
                fd, path = tempfile.mkstemp(suffix=".wav", prefix="ha_tts_")
                os.close(fd)
                subprocess.run([exe, "-w", path, text], capture_output=True, timeout=30)
                return path
            except Exception:
                pass
        return None


if __name__ == "__main__":
    # Bring-up self-check: resolve the headset, play a TTS line, capture an utterance.
    import argparse

    ap = argparse.ArgumentParser(description="Bluetooth-headset I/O self-check.")
    ap.add_argument("--no-tts", action="store_true")
    ap.add_argument("--seconds", type=float, default=6.0)
    args = ap.parse_args()

    hs = Headset()
    print("Endpoint:", hs.ep.describe())
    print("Info:", json.dumps(hs.info(), indent=2))
    if not args.no_tts:
        hs.say("Bluetooth headset check. Please say something after the tone.")
    pcm = hs.record_utterance(max_seconds=args.seconds)
    print(f"Captured {len(pcm)} PCM bytes ({len(pcm)/(RATE*WIDTH):.1f}s), RMS={_rms(pcm):.4f}")
