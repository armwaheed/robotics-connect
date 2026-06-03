#!/usr/bin/env python3
"""TouchEvent — per-hand "any-finger-touched" first-contact detector.

The brainco_bridge exposes a 60-reg rich touch sensor block at Modbus
reg 4300+ (empirically verified on Revo2Touch).  Each press —
on any of the 10 fingers across both hands — fires ~36 of those 60 regs
above |Δ| > 1024 from idle baseline.  Per-finger register mapping is
NOT static (mechanical cross-talk through the rigid hand chassis), so
this module treats each hand's 60-reg block as a single per-hand
"any-finger-touched" scalar:

    fire if  max(|reg - baseline|)  >  threshold

Idle baseline is captured at `arm()` time and held until next `arm()`.
The `fired()` call returns (l_hit, r_hit, l_peak_delta, r_peak_delta).

Replaces the earlier force-threshold pattern
    `if l_touch[ti] >= FINGER_CONTACT_THRESHOLD and ...`
which used the 30-reg block at 4200+ (force-only, doesn't see thumb /
pinky reliably in V2 firmware) and fired at 0.8 normalised force ≈ 400
raw — much later than first contact.  This module fires at first tap
because the 4300+ encoding saturates fast (each press jumps the high
byte from idle ~50 to ~248 in a fraction of a second).

API:

    te = TouchEvent(hand_client, threshold=5000)
    te.arm()                  # capture fresh baseline of both hands' 60-reg blocks
    while ...:
        l_hit, r_hit, lp, rp = te.fired()
        if l_hit or r_hit:
            ...

Threshold tuning:
  - 5000 raw is "light tap" — well above noise floor (~few hundred raw
    in idle measurements) and well below the saturation deltas of
    ~50000 raw seen in the 2026-05-13 press tests.
  - Drop to 2000 for hair-trigger / very lightweight objects.
  - Bump to 10000 if false positives from arm motion shake-up.
"""
from __future__ import annotations

from typing import Tuple


DEFAULT_THRESHOLD = 5000  # raw u16 deflection that counts as first contact


class TouchEvent:
    """Per-hand any-finger-touched detector.

    Caller is responsible for calling arm() to capture a fresh baseline
    before any motion that should be monitored.  fired() is non-blocking
    and queries the bridge once per call — at 10 Hz this is well within
    the bridge's TCP RPC throughput.
    """

    def __init__(self, hand_client, threshold: int = DEFAULT_THRESHOLD):
        self._client = hand_client
        self._threshold = int(threshold)
        self._baseline_l = None
        self._baseline_r = None
        self._armed = False

    @property
    def threshold(self) -> int:
        return self._threshold

    def arm(self) -> None:
        """Capture a fresh baseline of both hands' 60-reg blocks.

        Call right before the motion you want to monitor — e.g. just
        before the close-test ramp starts, or right before the servo
        descent begins.
        """
        l, r = self._client.get_touch60()
        self._baseline_l = list(l)
        self._baseline_r = list(r)
        self._armed = True

    def fired(self) -> Tuple[bool, bool, int, int]:
        """Return (l_hit, r_hit, l_peak_delta, r_peak_delta).

        Each peak_delta is the max |reg - baseline| across the 60-reg
        block.  l_hit / r_hit fire when their peak_delta exceeds the
        configured threshold.  Returns (False, False, 0, 0) when not
        armed yet.
        """
        if not self._armed:
            return False, False, 0, 0
        l, r = self._client.get_touch60()
        l_peak = self._peak_delta(l, self._baseline_l)
        r_peak = self._peak_delta(r, self._baseline_r)
        return (
            l_peak >= self._threshold,
            r_peak >= self._threshold,
            l_peak,
            r_peak,
        )

    @staticmethod
    def _peak_delta(now, baseline) -> int:
        if not baseline or len(baseline) != len(now):
            return 0
        peak = 0
        for n, b in zip(now, baseline):
            d = abs(int(n) - int(b))
            if d > peak:
                peak = d
        return peak
