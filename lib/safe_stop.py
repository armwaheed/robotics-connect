"""Guaranteed-damp safety wrapper for any process that commands a real robot's motors.

The cardinal rule (see ../SAFETY.md): **never hard-kill a low-level control process.** When the
motor-command publisher dies without a final safe command, the robot keeps acting on the last
position target *at the configured stiffness* — on a humanoid at transfer gains that is a violent
runaway. A `SIGKILL` cannot be caught, so a process can't damp on the way out; that is precisely
why you never use it as a stop.

`SafeStop` removes every *catchable* unsafe exit: it damps the robot on normal return, on an
exception, and on `SIGINT`/`SIGTERM`/`SIGHUP`, then lets the process die. It cannot help against
`SIGKILL` or a power loss — those are why a hardware e-stop and a controller firmware-damp must
always be in reach (SAFETY.md §1).

Robot-agnostic: you provide a ``damp_fn`` that issues ONE compliant command (zero stiffness,
light damping, zero feed-forward torque) across all joints; `SafeStop` sustains and repeats it.
The Unitree G1 binding lives in ``unitree/g1`` (publish a ``rt/lowcmd`` with every
``motor_cmd[i].kp = 0, kd ≈ 3, q = measured, tau = 0``).

Usage (must be entered on the MAIN thread so signals are delivered):

    from lib.safe_stop import SafeStop

    def damp_once():
        g1.publish_damp()          # one rt/lowcmd: kp=0, kd=3, tau=0, q=measured

    with SafeStop(damp_once, name="bedreach"):
        run_control_loop()         # 50 Hz; poll the controller abort each tick

Standalone panic-damp — wire ``damp_fn`` to the robot and call :func:`panic_damp` from a SECOND
shell to make a runaway inert WITHOUT approaching it. Tests: ``python3 test_safe_stop.py``.
"""
from __future__ import annotations

import atexit
import signal
import sys
import threading
import time
from typing import Callable, Optional

DampFn = Callable[[], None]


def panic_damp(damp_fn: DampFn, seconds: float = 3.0, hz: float = 50.0,
               verbose: bool = True) -> None:
    """Flood a compliant damp for ``seconds``. A zero-stiffness damp can only make the joints
    compliant — it can NOT drive a posture — so this is always safe to run, including against a
    robot that is already moving. Run it from a separate shell to safe a robot remotely.

    ``damp_fn`` must issue ONE compliant command per call (non-blocking)."""
    n = max(1, int(seconds * hz))
    dt = 1.0 / hz
    if verbose:
        print(f"[panic_damp] flooding compliant damp for {seconds:.1f}s @ {hz:.0f} Hz",
              file=sys.stderr)
    for _ in range(n):
        try:
            damp_fn()
        except Exception as e:  # never let the safety path raise
            print(f"[panic_damp] damp_fn raised: {e!r}", file=sys.stderr)
        time.sleep(dt)


class SafeStop:
    """Context manager that guarantees a damp on every *catchable* exit.

    Args:
        damp_fn:    issues ONE compliant command across all joints (non-blocking).
        name:       label for log lines.
        reengage_fn: optional — re-select the vendor balance controller after damping
                    (only safe if the robot is still upright; omit for a supported/fallen robot).
        hold_s/hz:  how long / how fast to repeat ``damp_fn`` when damping (sustains it on the bus).
        verbose:    log to stderr.

    Enter on the MAIN thread (signals are only delivered there). The damp runs at most once.
    """

    def __init__(self, damp_fn: DampFn, *, name: str = "control",
                 reengage_fn: Optional[Callable[[], None]] = None,
                 hold_s: float = 1.0, hz: float = 50.0, verbose: bool = True) -> None:
        self._damp = damp_fn
        self._reengage = reengage_fn
        self._name = name
        self._hold_s = hold_s
        self._hz = hz
        self._verbose = verbose
        self._done = threading.Event()
        self._prev: dict = {}

    # ── damping ──────────────────────────────────────────────────────────────
    def damp(self, reason: str = "manual") -> None:
        """Sustain the compliant damp. Idempotent — runs at most once per instance."""
        if self._done.is_set():
            return
        self._done.set()
        if self._verbose:
            print(f"[SafeStop:{self._name}] DAMPING (reason: {reason})", file=sys.stderr)
        panic_damp(self._damp, seconds=self._hold_s, hz=self._hz, verbose=False)
        if self._reengage is not None:
            try:
                self._reengage()
            except Exception as e:
                print(f"[SafeStop:{self._name}] reengage_fn raised: {e!r}", file=sys.stderr)

    # ── signal / lifecycle plumbing ──────────────────────────────────────────
    def _on_signal(self, signum, _frame):
        self.damp(f"signal {signal.Signals(signum).name}")
        self._restore()
        signal.raise_signal(signum)  # re-deliver to the (now default) handler → process exits

    def _restore(self) -> None:
        for sig, handler in self._prev.items():
            try:
                signal.signal(sig, handler)
            except Exception:
                pass
        self._prev.clear()

    def __enter__(self) -> "SafeStop":
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            try:
                self._prev[sig] = signal.signal(sig, self._on_signal)
            except (ValueError, OSError):
                # not main thread, or signal unsupported on this platform — still have
                # __exit__ + atexit covering normal/exception exits.
                pass
        atexit.register(self._atexit)
        return self

    def __exit__(self, exc_type, exc, tb) -> bool:
        self.damp("normal exit" if exc_type is None else f"exception {exc_type.__name__}")
        self._restore()
        return False  # never suppress the exception

    def _atexit(self) -> None:
        self.damp("atexit")


__all__ = ["SafeStop", "panic_damp"]
