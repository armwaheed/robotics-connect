"""Humanoid locomotion — a robot-agnostic velocity-walk + waypoint layer.

Any humanoid that can (a) accept a body-frame velocity command ``(vx, vy, vyaw)``
and (b) report a *measured* planar pose can plug a concrete :class:`LocomotionController`
subclass in, and the closed-loop helpers (:meth:`walk_to`, :meth:`turn_to`,
:meth:`walk_forward`) work unchanged.

Conventions (REP-103, robot body frame): ``+x`` forward, ``+y`` left, ``+yaw``
counter-clockwise; metres and radians throughout. The pose is in whatever fixed
odometry frame the concrete controller reports — the helpers only ever use pose
*differences*, so the frame's origin is irrelevant.

The robotics-connect G1 binding lives in ``unitree/g1/locomotion``;
:class:`SimLocomotion` below is a dependency-free kinematic model used by the
offline tests and for off-robot dry-runs.
"""

from __future__ import annotations

import abc
import math
import time
from dataclasses import dataclass

# ── Tunables ────────────────────────────────────────────────────────────────
CONTROL_HZ = 50.0           # closed-loop tick rate
CMD_PERIOD_S = 0.10         # min interval between re-issued velocity commands
WALK_SPEED = 0.30           # m/s, nominal approach speed
WALK_TOLERANCE_M = 0.10     # waypoint arrival radius
STEP_TOLERANCE_M = 0.05     # tighter radius for short precise steps
TURN_TOLERANCE_RAD = 0.10   # heading arrival tolerance
YAW_HOLD_GAIN = 0.8         # P-gain on heading error (rad/s per rad)
YAW_RATE_MAX = 0.40         # rad/s, clamp on commanded turn rate
WALK_TIMEOUT_S = 30.0
TURN_TIMEOUT_S = 15.0


def wrap_pi(angle: float) -> float:
    """Wrap an angle to ``(-pi, pi]``."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class Pose:
    """Planar pose in the controller's odometry frame (metres, radians)."""

    x: float
    y: float
    yaw: float


class LocomotionController(abc.ABC):
    """Abstract velocity-walk + waypoint interface for a humanoid.

    A concrete robot implements three primitives — :meth:`set_velocity`,
    :meth:`pose`, :meth:`stop` — and inherits the closed-loop helpers. Override
    the lifecycle hooks (:meth:`stand`, :meth:`damp`, :meth:`connect`,
    :meth:`shutdown`) and :meth:`is_blocked` where the platform supports them.
    """

    # ── Primitives a concrete controller MUST provide ───────────────────────
    @abc.abstractmethod
    def set_velocity(self, vx: float, vy: float, vyaw: float) -> None:
        """Command a body-frame velocity (m/s, m/s, rad/s)."""

    @abc.abstractmethod
    def pose(self) -> Pose:
        """Return the latest *measured* planar pose."""

    @abc.abstractmethod
    def stop(self) -> None:
        """Bring the robot to a halt (zero velocity), staying balanced."""

    # ── Optional lifecycle / safety hooks (safe no-op defaults) ─────────────
    def connect(self) -> None:
        """Acquire control of the robot. Default: nothing to do."""

    def stand(self) -> None:
        """Bring the robot to a stable balanced stand. Default: no-op."""

    def damp(self) -> None:
        """Drop to a soft damping state. Default: delegates to :meth:`stop`."""
        self.stop()

    def shutdown(self) -> None:
        """Release control and stop background work. Default: :meth:`stop`."""
        self.stop()

    def is_blocked(self, commanded_vx: float) -> str | None:
        """Return a reason string if the robot is stalled/obstructed, else None.

        ``commanded_vx`` is the forward speed the follower is asking for, so a
        platform can compare intended vs. achieved motion. Default: never blocks.
        """
        return None

    # ── Convenience velocity verbs ──────────────────────────────────────────
    def forward(self, vx: float) -> None:
        self.set_velocity(vx, 0.0, 0.0)

    def strafe(self, vy: float) -> None:
        self.set_velocity(0.0, vy, 0.0)

    def turn(self, vyaw: float) -> None:
        self.set_velocity(0.0, 0.0, vyaw)

    # ── Closed-loop helpers (use the measured pose) ─────────────────────────
    def walk_to(
        self,
        xy: tuple[float, float],
        tolerance_m: float = WALK_TOLERANCE_M,
        vmax: float = WALK_SPEED,
        timeout_s: float = WALK_TIMEOUT_S,
        hold_heading: bool = True,
        stall_guard: bool = True,
    ) -> str | None:
        """Walk to odometry-frame ``xy``, holding the start heading by default.

        Returns ``None`` on arrival, ``"timeout"`` if the budget expires, or the
        :meth:`is_blocked` reason if the stall guard fires. :meth:`stop` is
        always issued before returning.
        """
        target_x, target_y = float(xy[0]), float(xy[1])
        heading = self.pose().yaw if hold_heading else None
        deadline = time.monotonic() + timeout_s
        tick = 1.0 / CONTROL_HZ
        last_cmd = 0.0

        while time.monotonic() < deadline:
            p = self.pose()
            dx, dy = target_x - p.x, target_y - p.y
            dist = math.hypot(dx, dy)
            if dist <= tolerance_m:
                self.stop()
                return None

            if stall_guard:
                reason = self.is_blocked(vmax)
                if reason is not None:
                    self.stop()
                    return reason

            # Rotate the world-frame unit direction into the body frame.
            cos_y, sin_y = math.cos(p.yaw), math.sin(p.yaw)
            ux, uy = dx / dist, dy / dist
            vx = (ux * cos_y + uy * sin_y) * vmax
            vy = (-ux * sin_y + uy * cos_y) * vmax
            vyaw = self._heading_correction(p.yaw, heading)

            now = time.monotonic()
            if now - last_cmd >= CMD_PERIOD_S:
                self.set_velocity(vx, vy, vyaw)
                last_cmd = now
            time.sleep(tick)

        self.stop()
        return "timeout"

    def walk_forward(self, distance_m: float, **kwargs) -> str | None:
        """Walk ``distance_m`` straight ahead along the current heading."""
        p = self.pose()
        target = (p.x + distance_m * math.cos(p.yaw),
                  p.y + distance_m * math.sin(p.yaw))
        return self.walk_to(target, **kwargs)

    def step_to(self, xy: tuple[float, float], **kwargs) -> str | None:
        """A short, precise placement step: tight tolerance, no stall guard."""
        kwargs.setdefault("tolerance_m", STEP_TOLERANCE_M)
        kwargs.setdefault("stall_guard", False)
        return self.walk_to(xy, **kwargs)

    def turn_to(
        self,
        target_yaw: float,
        tolerance_rad: float = TURN_TOLERANCE_RAD,
        timeout_s: float = TURN_TIMEOUT_S,
    ) -> str | None:
        """Rotate in place until the measured yaw reaches ``target_yaw``."""
        deadline = time.monotonic() + timeout_s
        tick = 1.0 / CONTROL_HZ
        last_cmd = 0.0

        while time.monotonic() < deadline:
            err = wrap_pi(target_yaw - self.pose().yaw)
            if abs(err) <= tolerance_rad:
                self.stop()
                return None
            vyaw = _clamp(YAW_HOLD_GAIN * err, YAW_RATE_MAX)
            now = time.monotonic()
            if now - last_cmd >= CMD_PERIOD_S:
                self.set_velocity(0.0, 0.0, vyaw)
                last_cmd = now
            time.sleep(tick)

        self.stop()
        return "timeout"

    # ── Internals ───────────────────────────────────────────────────────────
    @staticmethod
    def _heading_correction(yaw: float, hold: float | None) -> float:
        """P-controlled yaw rate that holds ``hold`` (or 0 if not holding)."""
        if hold is None:
            return 0.0
        err = wrap_pi(yaw - hold)
        if abs(err) < TURN_TOLERANCE_RAD:
            return 0.0
        return _clamp(-YAW_HOLD_GAIN * err, YAW_RATE_MAX)


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class SimLocomotion(LocomotionController):
    """Dependency-free kinematic model — integrates the commanded body velocity.

    Legitimate precisely because it *is* the robot: with no hardware there is
    nothing to measure, so the commanded velocity is ground truth. Used by the
    offline tests and for off-robot dry-runs of behaviour code.

    ``blocks_after_m`` makes :meth:`is_blocked` fire once cumulative travel
    reaches that distance, to exercise stall handling without hardware.
    """

    def __init__(self, blocks_after_m: float | None = None) -> None:
        self._vx = self._vy = self._vyaw = 0.0
        self._x = self._y = self._yaw = 0.0
        self._travelled = 0.0
        self._last_t: float | None = None
        self._blocks_after_m = blocks_after_m

    def _integrate(self) -> None:
        now = time.monotonic()
        if self._last_t is None:
            self._last_t = now
            return
        dt = now - self._last_t
        self._last_t = now
        if dt <= 0.0:
            return
        cos_y, sin_y = math.cos(self._yaw), math.sin(self._yaw)
        self._x += (self._vx * cos_y - self._vy * sin_y) * dt
        self._y += (self._vx * sin_y + self._vy * cos_y) * dt
        self._yaw = wrap_pi(self._yaw + self._vyaw * dt)
        self._travelled += math.hypot(self._vx, self._vy) * dt

    def set_velocity(self, vx: float, vy: float, vyaw: float) -> None:
        self._integrate()
        self._vx, self._vy, self._vyaw = vx, vy, vyaw

    def pose(self) -> Pose:
        self._integrate()
        return Pose(self._x, self._y, self._yaw)

    def stop(self) -> None:
        self.set_velocity(0.0, 0.0, 0.0)

    def is_blocked(self, commanded_vx: float) -> str | None:
        self._integrate()
        if self._blocks_after_m is not None and self._travelled >= self._blocks_after_m:
            return "blocked"
        return None
