#!/usr/bin/env python3
"""Offline tests for the closed-loop helpers, on SimLocomotion. No hardware.

Run: ``python3 test_locomotion.py``
"""
from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from locomotion import SimLocomotion  # noqa: E402

FAST = 1.0  # m/s — bump the speed so tests finish quickly


def _at(loco, x, y, tol=0.15):
    p = loco.pose()
    assert math.hypot(p.x - x, p.y - y) <= tol, f"at ({p.x:.2f}, {p.y:.2f}), want ({x}, {y})"


def test_walk_to_forward():
    loco = SimLocomotion()
    assert loco.walk_to((1.0, 0.0), vmax=FAST, timeout_s=8.0) is None
    _at(loco, 1.0, 0.0)


def test_walk_to_diagonal():
    loco = SimLocomotion()
    assert loco.walk_to((1.0, 1.0), vmax=FAST, timeout_s=10.0) is None
    _at(loco, 1.0, 1.0)


def test_walk_to_behind_left():
    loco = SimLocomotion()
    assert loco.walk_to((-0.5, 0.5), vmax=FAST, timeout_s=8.0) is None
    _at(loco, -0.5, 0.5)


def test_walk_forward_uses_heading():
    loco = SimLocomotion()
    assert loco.walk_forward(1.0, vmax=FAST, timeout_s=8.0) is None
    _at(loco, 1.0, 0.0)


def test_turn_to_rotates_in_place():
    loco = SimLocomotion()
    assert loco.turn_to(0.6, timeout_s=8.0) is None
    assert abs(loco.pose().yaw - 0.6) <= 0.1
    _at(loco, 0.0, 0.0, tol=0.05)  # turning must not translate


def test_step_to_tight_tolerance():
    loco = SimLocomotion()
    assert loco.step_to((0.20, 0.0), vmax=0.3, timeout_s=5.0) is None
    _at(loco, 0.20, 0.0, tol=0.07)


def test_stall_guard_fires():
    loco = SimLocomotion(blocks_after_m=1.5)
    res = loco.walk_to((3.0, 0.0), vmax=FAST, timeout_s=10.0, stall_guard=True)
    assert res == "blocked", f"expected 'blocked', got {res!r}"
    assert 1.2 <= loco.pose().x <= 1.9


def test_timeout():
    loco = SimLocomotion()
    res = loco.walk_to((50.0, 0.0), vmax=FAST, timeout_s=1.0)
    assert res == "timeout", f"expected 'timeout', got {res!r}"


TESTS = [v for k, v in sorted(globals().items()) if k.startswith("test_")]


def main() -> int:
    failures = 0
    for t in TESTS:
        try:
            t()
            print(f"  PASS  {t.__name__}")
        except AssertionError as exc:
            failures += 1
            print(f"  FAIL  {t.__name__}: {exc}")
    print(f"\n{len(TESTS) - failures}/{len(TESTS)} passed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
