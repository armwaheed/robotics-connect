#!/usr/bin/env python3
"""Speak text through the Unitree G1 EDU's chest speaker.

Runs in the **SDK conda env** (the one with ``unitree_sdk2py`` + CycloneDDS), driving the chest
speaker via the Unitree ``AudioClient`` over the DDS "voice" service. It is invoked as a SUBPROCESS
by the Device Connect sidecar (``g1_agent.py``), which runs in a separate Python 3.11 env
(device-connect-edge requires >=3.11, the SDK env is 3.10) — so the DDS speaker code stays in its own
working env and nothing has to be cross-installed (the two-env bridge).

  SDK-env python g1_speak.py "Can you hold the far corner of the sheet for me?"

Volume is pinned to full (100) on every call so the robot is clearly audible — this is the OUT-LOUD
channel the robot uses to ask its human partner for help (armwaheed/robots#3).
"""

from __future__ import annotations

import os
import sys

# The deployed voice module on the robot (g1_voice.py: G1Speaker over the DDS "voice" service).
_VOICE_DIR = os.environ.get("G1_VOICE_DIR", "/home/unitree/robotics-connect/unitree/g1/voice")
sys.path.insert(0, _VOICE_DIR)


def main() -> int:
    if len(sys.argv) < 2:
        print("usage: g1_speak.py <text>", file=sys.stderr)
        return 2
    text = sys.argv[1]
    iface = os.environ.get("G1_DDS_IFACE", "eth0")
    try:
        from g1_voice import G1Speaker
    except Exception as exc:
        print(f"g1_voice unavailable ({exc!r})", file=sys.stderr)
        return 3
    # Pin full gain explicitly (don't depend on the deployed module's default), English voice.
    spk = G1Speaker(iface=iface, default_volume=100, speaker_id=4)
    spoke = spk.say(text, wait=True)
    print("spoke" if spoke else "console-fallback")
    return 0 if spoke else 1


if __name__ == "__main__":
    raise SystemExit(main())
