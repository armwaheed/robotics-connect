#!/usr/bin/env python3
"""Offline tests for the SafeStop damp-on-exit guarantee. No hardware.

Run: ``python3 test_safe_stop.py``
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from safe_stop import SafeStop, panic_damp  # noqa: E402

FAST = dict(hold_s=0.02, hz=50.0, verbose=False)


def test_damps_on_normal_exit():
    n = [0]
    with SafeStop(lambda: n.__setitem__(0, n[0] + 1), name="t", **FAST):
        pass
    assert n[0] > 0


def test_damps_on_exception():
    n = [0]
    try:
        with SafeStop(lambda: n.__setitem__(0, n[0] + 1), name="t", **FAST):
            raise RuntimeError("boom")
    except RuntimeError:
        pass
    assert n[0] > 0


def test_does_not_suppress_exception():
    raised = False
    try:
        with SafeStop(lambda: None, name="t", **FAST):
            raise ValueError("x")
    except ValueError:
        raised = True
    assert raised, "SafeStop must not swallow the exception"


def test_damp_is_idempotent():
    n = [0]
    s = SafeStop(lambda: n.__setitem__(0, n[0] + 1), name="t", **FAST)
    s.damp("a")
    first = n[0]
    s.damp("b")
    assert n[0] == first, "second damp() must be a no-op"


def test_panic_damp_floods():
    n = [0]
    panic_damp(lambda: n.__setitem__(0, n[0] + 1), seconds=0.1, hz=50.0, verbose=False)
    assert n[0] >= 3, "panic_damp must repeat the compliant command"


def test_damp_fn_exception_is_swallowed():
    # the safety path must never raise out
    def bad():
        raise RuntimeError("damp failed")
    with SafeStop(bad, name="t", **FAST):
        pass  # must not raise


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"  ok  {t.__name__}")
    print(f"safe_stop: {len(tests)}/{len(tests)} passed")
