#!/usr/bin/env python3
"""
Voice diagnostic for the G1 EDU — run ON THE ROBOT to confirm the audio I/O is live.

Checks, in order:
  1. SPEAKER  — initialise AudioClient, set volume, speak a test phrase via TtsMaker, blink the LED.
  2. MIC      — list PulseAudio sources (the mic-array shows up here; ALSA on the Tegra does not).
  3. ASR      — (optional) record a short utterance and transcribe it, to prove the full ask loop.

Usage:
  python _diag_voice.py --iface eth0                 # speaker + mic-source listing
  python _diag_voice.py --iface eth0 --listen        # also record 5 s and transcribe
  python _diag_voice.py --iface eth0 --ask           # full ask("...?", yes/no) round-trip

Exit code 0 iff the speaker initialised. The mic/ASR results are reported but do not fail the run
(the speaker is the SDK-supported half; the mic is best-effort OS capture).
"""

from __future__ import annotations

import argparse
import sys

from g1_voice import VoiceIO, list_pulse_sources, make_asr


def main() -> int:
    ap = argparse.ArgumentParser(description="G1 voice diagnostic.")
    ap.add_argument("--iface", default=None, help="DDS network interface to the robot (e.g. eth0).")
    ap.add_argument("--asr", default="auto", help="ASR backend: auto|whisper|vosk|manual")
    ap.add_argument("--listen", action="store_true", help="Record 5 s and transcribe.")
    ap.add_argument("--ask", action="store_true", help="Full ask() yes/no round-trip.")
    ap.add_argument("--phrase", default="Hello. I am the bed making robot. Can you hear me?")
    args = ap.parse_args()

    print("== 1. SPEAKER ==")
    vio = VoiceIO(iface=args.iface, asr_backend=args.asr)
    spoke = vio.speaker.say(args.phrase, wait=True)
    vio.speaker.led((0, 140, 0))
    print(f"   speaker available: {vio.speaker.available}   spoke via TtsMaker: {spoke}")

    print("\n== 2. MICROPHONE (PulseAudio sources) ==")
    sources = list_pulse_sources()
    if sources:
        for s in sources:
            print("   ", s)
    else:
        print("   (no PulseAudio sources found — off-robot, or select one with `pactl set-default-source`)")
    print(f"   parec capture available: {vio.mic.available}")

    if args.listen or args.ask:
        print("\n== 3. ASR ==")
        if args.ask:
            res = vio.ask("Can you help me with this corner? Please say yes or no.", choices="yesno")
            print(f"   heard={res.heard}  transcript={res.transcript!r}  ->  choice={res.choice!r} ({res.confidence:.2f})")
        else:
            vio.speaker.say("Please say something after the light turns blue.", wait=True)
            vio.speaker.led((0, 80, 160))
            pcm = vio.mic.record_utterance(max_seconds=5.0)
            vio.speaker.led((0, 0, 0))
            text = make_asr(args.asr)(pcm, vio.mic.rate) if pcm else "(no audio captured)"
            print(f"   captured {len(pcm)} PCM bytes  ->  transcript={text!r}")

    return 0 if vio.speaker.available else 1


if __name__ == "__main__":
    sys.exit(main())
