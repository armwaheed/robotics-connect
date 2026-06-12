"""Unitree G1 binding for the robot-agnostic locomotion layer.

:class:`G1Locomotion` drives the G1 through Unitree's high-level ``LocoClient``
— the manufacturer's balance controller — and reads **measured** odometry from
the ``rt/odommodestate`` DDS topic (``SportModeState_``: ``position``,
``velocity``, ``yaw_speed``). No reinforcement-learning policy is pushed onto
the legs; walking is the vendor's velocity-command interface, so balance is the
controller's responsibility.

Frames: body ``+x`` forward / ``+y`` left, planar pose in the estimator's odom
frame. The shared closed-loop helpers (``walk_to``, ``turn_to``,
``walk_forward``) live in ``lib/locomotion.py``.

Safety: ``set_velocity`` / ``stand`` move the legs. The caller is responsible
for a clear area, an operator on the e-stop, and adequate battery.
"""

from __future__ import annotations

import argparse
import importlib.util
import math
import sys
import threading
import time
from pathlib import Path


def _load_module(name: str, path: Path):
    """Load a module by file path under a unique name.

    The shared layer is ``lib/locomotion.py``; importing it as a bare
    ``locomotion`` would clash with any consumer that has its own
    ``locomotion`` module on ``sys.path`` (the bed-making demo does). Loading
    it by path under a unique key avoids that entirely.
    """
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


_locomotion = _load_module(
    "robotics_connect_locomotion",
    Path(__file__).resolve().parents[3] / "lib" / "locomotion.py",
)
LocomotionController = _locomotion.LocomotionController
Pose = _locomotion.Pose

ODOM_TOPIC = "rt/odommodestate"
STALL_SPEED_FRACTION = 0.30  # measured/commanded speed below this counts as stalled
STALL_GRACE_S = 1.5          # ...sustained for this long → blocked

_dds_ready = False


def _ensure_dds(iface: str) -> None:
    """Initialise the Cyclone DDS channel factory once per process."""
    global _dds_ready
    if _dds_ready:
        return
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    ChannelFactoryInitialize(0, iface)
    _dds_ready = True


class G1Locomotion(LocomotionController):
    """Velocity-walk + measured-odometry control of a Unitree G1."""

    def __init__(self, iface: str = "eth0", init_dds: bool = True) -> None:
        self._iface = iface
        self._init_dds = init_dds
        self._client = None
        self._sub = None
        self._lock = threading.Lock()
        self._odom = None            # latest SportModeState_
        self._stall_since: float | None = None

    # ── Lifecycle ───────────────────────────────────────────────────────────
    def connect(self) -> None:
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

        if self._init_dds:
            _ensure_dds(self._iface)

        self._client = LocoClient()
        self._client.SetTimeout(5.0)
        self._client.Init()

        self._sub = ChannelSubscriber(ODOM_TOPIC, SportModeState_)
        self._sub.Init(self._on_odom, 10)

        deadline = time.monotonic() + 3.0
        while self._odom is None and time.monotonic() < deadline:
            time.sleep(0.02)
        if self._odom is None:
            self.shutdown()  # don't leak the subscriber on the error path
            raise RuntimeError(f"no {ODOM_TOPIC} in 3 s — is the robot on?")
        p = self.pose()
        print(f"[G1Locomotion] odom live  pose=({p.x:+.2f}, {p.y:+.2f}, "
              f"{math.degrees(p.yaw):+.1f}°)")

    def _on_odom(self, msg) -> None:
        with self._lock:
            self._odom = msg

    def shutdown(self) -> None:
        try:
            self.stop()  # never let cleanup raise (e.g. called from connect()'s error path)
        except Exception:
            pass
        if self._sub is not None:
            try:
                self._sub.Close()
            except Exception:
                pass
            self._sub = None

    def stand(self) -> None:
        """Bring the robot to a stable balanced stand (legs take the weight)."""
        self._client.BalanceStand()

    def damp(self) -> None:
        """Drop to soft damping. Collapses the robot — only when supported."""
        self._client.Damp()

    # ── Primitives ──────────────────────────────────────────────────────────
    def set_velocity(self, vx: float, vy: float, vyaw: float) -> None:
        self._client.Move(vx, vy, vyaw)

    def stop(self) -> None:
        if self._client is not None:
            self._client.StopMove()

    def pose(self) -> Pose:
        with self._lock:
            odom = self._odom
        if odom is None:
            return Pose(0.0, 0.0, 0.0)
        pos = odom.position
        yaw = float(odom.imu_state.rpy[2])
        return Pose(float(pos[0]), float(pos[1]), yaw)

    def velocity(self) -> tuple[float, float, float]:
        """Measured body-frame velocity ``(vx, vy, vyaw)`` from the estimator."""
        with self._lock:
            odom = self._odom
        if odom is None:
            return (0.0, 0.0, 0.0)
        return (float(odom.velocity[0]), float(odom.velocity[1]),
                float(odom.yaw_speed))

    def is_blocked(self, commanded_vx: float) -> str | None:
        """Stalled if commanded to move but the measured speed stays near zero."""
        if commanded_vx <= 0.05:
            self._stall_since = None
            return None
        vx, vy, _ = self.velocity()
        moving = math.hypot(vx, vy) >= STALL_SPEED_FRACTION * commanded_vx
        if moving:
            self._stall_since = None
            return None
        now = time.monotonic()
        if self._stall_since is None:
            self._stall_since = now
        elif now - self._stall_since >= STALL_GRACE_S:
            self._stall_since = None
            return "stalled"
        return None


def main() -> None:
    ap = argparse.ArgumentParser(description="G1 locomotion diagnostics.")
    ap.add_argument("--iface", default="eth0", help="DDS network interface")
    ap.add_argument("--seconds", type=float, default=8.0,
                    help="how long to stream the measured pose (read-only)")
    ap.add_argument("--forward", type=float, default=0.0,
                    help="DANGER: walk this many metres forward, then stop")
    args = ap.parse_args()

    loco = G1Locomotion(iface=args.iface)
    loco.connect()

    if args.forward > 0.0:
        reply = input(f"About to WALK {args.forward:.2f} m forward — the legs "
                      f"will move. Clear area + e-stop ready? [type 'walk']: ")
        if reply.strip().lower() != "walk":
            print("aborted.")
            return
        loco.stand()
        result = loco.walk_forward(args.forward)
        loco.stop()
        print(f"[walk_forward] result={result!r}  pose={loco.pose()}")
        return

    print(f"streaming measured pose for {args.seconds:.0f}s (no motion)…")
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        p = loco.pose()
        vx, vy, vyaw = loco.velocity()
        print(f"  pose=({p.x:+.2f}, {p.y:+.2f}, {math.degrees(p.yaw):+6.1f}°)  "
              f"vel=({vx:+.2f}, {vy:+.2f}, {vyaw:+.2f})")
        time.sleep(0.5)
    loco.shutdown()


if __name__ == "__main__":
    main()
