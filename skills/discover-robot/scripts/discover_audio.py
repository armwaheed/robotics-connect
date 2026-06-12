#!/usr/bin/env python3
"""Discover a robot's audio I/O into the descriptor's `audio` block (speaker + mic + ASR).

Vendor-neutral. A humanoid may speak through an SDK audio service, a plain ALSA codec, or not at all;
its microphone may be a normal capture device, a DDS topic, or a CLOSED off-board stream that the
firmware ships to a vendor app/cloud with no local userspace hook. This probes for each and emits the
`audio` block plus a `recommended_listen_path` — onboard if the mic is exposed, else a local (USB/ALSA)
mic, else route a HUMAN in as a Device Connect agent (the path the bed-making G1 uses, because its mic
is a closed off-board stream).

Probes (best-effort, each degrades gracefully):
  * speaker — an SDK audio client (e.g. Unitree AudioClient over the DDS "voice" service)? else `aplay -l`.
  * mic     — `arecord -l` capture codecs; whether the ONLY nodes are Tegra APE/XBAR *virtual* devices
              (the closed-system signature: capture exists but isn't a userspace codec); a USB mic.
  * asr     — an on-board ASR process/service reachable by your code (default: none).

Usage:
  python discover_audio.py                 # probe this host
  python discover_audio.py --sdk unitree   # apply Unitree AudioClient knowledge for the speaker
  python discover_audio.py --mock          # emit the block shape with no hardware probes
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess

# Tegra audio-fabric virtual nodes — capture that exists in ALSA but is not a userspace codec.
VIRTUAL_HINTS = ("APE", "XBAR", "ADMAIF", "tegra", "DMIC", "I2S")


def _alsa_cards(tool: str) -> list[str]:
    """`card N: ...` lines from `arecord -l` (capture) or `aplay -l` (playback)."""
    if not shutil.which(tool):
        return []
    try:
        out = subprocess.run([tool, "-l"], capture_output=True, text=True, timeout=5).stdout
    except Exception:
        return []
    return [ln.strip() for ln in out.splitlines() if ln.strip().lower().startswith("card ")]


def _only_virtual(cards: list[str]) -> bool:
    """True if every capture card looks like a Tegra audio-fabric virtual node (not a real codec)."""
    return bool(cards) and all(any(h.lower() in c.lower() for h in VIRTUAL_HINTS) for c in cards)


def _has_usb_audio(cards: list[str]) -> bool:
    return any("usb" in c.lower() for c in cards)


def _sdk_speaker(sdk: str | None) -> dict | None:
    """Speaker dict from a known SDK audio client, or None to fall back to ALSA."""
    if sdk == "unitree":
        try:
            __import__("unitree_sdk2py.g1.audio.g1_audio_client")
            tail = ""
        except Exception:
            tail = " [SDK not importable on this host]"
        return {
            "present": True,
            "api": "Unitree AudioClient (DDS \"voice\" service)" + tail,
            "tts": True,
            "tts_voices": {"0": "zh", "1-4": "en"},
            "pcm_format": "16kHz/mono/16-bit LE (PlayStream)",
            "notes": "TtsMaker built-in TTS; PlayStream for bring-your-own PCM; SetVolume = persistent master gain.",
        }
    return None


def discover(sdk: str | None = None, mock: bool = False) -> dict:
    capture = [] if mock else _alsa_cards("arecord")
    playback = [] if mock else _alsa_cards("aplay")

    # ── speaker ──────────────────────────────────────────────────────────────
    speaker = _sdk_speaker(sdk)
    if speaker is None:
        speaker = {"present": bool(playback), "api": "ALSA aplay" if playback else "none", "tts": False}

    # ── microphone ───────────────────────────────────────────────────────────
    mic = {"present": False, "exposed_to_userspace": False, "access": "none"}
    if _has_usb_audio(capture):
        mic.update(present=True, exposed_to_userspace=True, access="usb",
                   notes="USB audio capture device — readable as a normal ALSA/PulseAudio source.")
    elif capture and not _only_virtual(capture):
        mic.update(present=True, exposed_to_userspace=True, access="alsa_codec",
                   notes="A real ALSA capture codec is present.")
    elif capture and _only_virtual(capture):
        mic.update(present=True, exposed_to_userspace=False, access="closed_offboard_stream",
                   notes="Only Tegra APE/XBAR virtual capture nodes — no userspace codec. If the vendor "
                         "firmware opens the mic and streams it off-robot, it is a closed system.")
    # else: no capture cards seen (off-robot / --mock) → leave present=false, access='none'.

    # ── asr (default: none on-board; override if you find a reachable ASR service) ──
    asr = {"onboard": False}

    # ── recommended listen path ──────────────────────────────────────────────
    if asr["onboard"]:
        path = "onboard"
    elif mic["access"] in ("usb", "alsa_codec"):
        path = "usb"
    elif mic["access"] == "dds_topic":
        path = "dds_topic"
    else:
        path = "device_connect_human_agent"
    asr["path"] = {
        "onboard": "on-board ASR reachable from your code",
        "usb": "a local capture device (USB/ALSA) + local ASR (e.g. faster-whisper)",
        "dds_topic": "subscribe the mic DDS topic + local ASR",
        "device_connect_human_agent": "route a human in as a Device Connect agent (headset + local ASR), "
                                      "or attach an external USB mic array",
    }[path]

    return {"speaker": speaker, "microphone": mic, "asr": asr, "recommended_listen_path": path}


def main() -> None:
    ap = argparse.ArgumentParser(description="Discover the robot's audio I/O (speaker + mic + ASR).")
    ap.add_argument("--sdk", default=None, help="Apply an SDK's audio knowledge for the speaker (e.g. 'unitree').")
    ap.add_argument("--mock", action="store_true", help="Emit the block shape with no hardware probes.")
    args = ap.parse_args()
    print(json.dumps(discover(sdk=args.sdk, mock=args.mock), indent=2))


if __name__ == "__main__":
    main()
