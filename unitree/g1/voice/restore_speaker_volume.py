#!/usr/bin/env python3
"""Restore the Unitree G1 EDU speaker to full/default gain.

A prior session lowered the speaker volume via ``AudioClient.SetVolume``. That call sets the
robot's MASTER speaker gain, which persists and softens *everything* — including the factory
mode-switch announcements. This reads the current volume, sets it back to full (default 100),
confirms, and speaks a test phrase so you can verify by ear.

Run ON the robot (or anywhere the DDS "voice" service is reachable):
    python restore_speaker_volume.py --iface eth0            # default: set to 100
    python restore_speaker_volume.py --iface eth0 --volume 90 # or a specific level
"""

from __future__ import annotations

import argparse
import sys


def main() -> int:
    ap = argparse.ArgumentParser(description="Restore the G1 speaker to full/default volume.")
    ap.add_argument("--iface", default="eth0", help="DDS interface to the robot (eth0 on the EDU).")
    ap.add_argument("--domain", type=int, default=0)
    ap.add_argument("--volume", type=int, default=100, help="Target master volume 0-100 (default 100).")
    ap.add_argument("--no-test", action="store_true", help="Skip the spoken test phrase.")
    args = ap.parse_args()

    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
    except Exception as exc:
        print(f"unitree_sdk2py not available ({exc!r}). Run this on the robot.", file=sys.stderr)
        return 2

    ChannelFactoryInitialize(args.domain, args.iface)
    client = AudioClient()
    client.SetTimeout(10.0)
    client.Init()

    try:
        code, before = client.GetVolume()
        print(f"GetVolume (before): code={code} volume={before}")
    except Exception as exc:
        print(f"GetVolume failed (continuing): {exc!r}")

    target = max(0, min(100, int(args.volume)))
    code = client.SetVolume(target)
    print(f"SetVolume({target}): code={code}")

    try:
        code, after = client.GetVolume()
        print(f"GetVolume (after):  code={code} volume={after}")
    except Exception as exc:
        print(f"GetVolume failed: {exc!r}")

    if not args.no_test:
        client.TtsMaker(f"Speaker volume restored to {target}. Can you hear me clearly now?", 4)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
