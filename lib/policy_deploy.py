"""Robot-agnostic deployment of a sim-trained policy onto real hardware — the de-risk ladder.

Running a policy in a control loop is not the same as putting it on real motors. A whole-body
policy drives the legs, so deploying it means taking the legs off the vendor balance controller —
a first transfer falls fast if anything is off. This module factors the parts that are the SAME
for every humanoid:

  * the **deploy contract** (joint order / scales / default offsets / obs term order / gains),
    dumped from the EXACT training env so sim↔real obs+action parity is exact (the #1 footgun);
  * the **observation builder** (term-major, from live robot state);
  * the **de-risk ladder** — :meth:`run_offline` (no motion) → :meth:`run_partial` (a fall-safe
    joint subset, e.g. arms over the vendor balance) → :meth:`run_whole` (full takeover);
  * **SafeStop baked into every motion stage**, so every exit damps (see ``safe_stop`` + SAFETY.md).

A robot provides a :class:`RobotIO` (read state, publish targets, damp, release the vendor
controller, report the controller abort). Everything else is shared. The Unitree G1 binding wraps
``rt/lowstate`` / ``rt/lowcmd`` / ``rt/arm_sdk`` / ``MotionSwitcher``; ``g1_bedreach_deploy.py`` in
armwaheed/robots#3 is the reference implementation this generalizes.

NEVER ``kill -9`` a process running these stages — use the controller abort or SIGTERM (which
SafeStop turns into a damp). A hard kill latches the last high-gain command → runaway (SAFETY.md §0).
"""
from __future__ import annotations

import json
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Callable, Optional

import numpy as np

try:                                            # works imported as a package or run as a script
    from .safe_stop import SafeStop
except ImportError:                             # pragma: no cover
    from safe_stop import SafeStop


# ── deploy contract ──────────────────────────────────────────────────────────
@dataclass
class DeployContract:
    """The sim↔real parity contract, dumped from the training env (not guessed from a descriptor).

    ``action_joint_names`` is the order the policy's outputs map to — IsaacLab interleaves the
    DOFs, so this is NOT the SDK index order; map to hardware **by name**."""

    action_joint_names: list
    action_scale: np.ndarray          # target = offset + scale * action
    action_offset: np.ndarray         # == default joint pos, action order
    joint_pos_default: np.ndarray     # obs joint_pos offset (== action_offset for these joints)
    obs_term_order: list              # e.g. [base_lin_vel, base_ang_vel, projected_gravity, <cmd>, joint_pos, joint_vel, actions]
    obs_total_dim: int
    control_hz: float
    gains: dict = field(default_factory=dict)   # joint-name -> (kp, kd); optional, for whole-body

    @classmethod
    def load(cls, path: str) -> "DeployContract":
        with open(path) as f:
            d = json.load(f)
        act = d["action"]
        obs = d["obs"]
        return cls(
            action_joint_names=list(act["joint_names_in_order"]),
            action_scale=np.asarray(act["scale"], dtype=float),
            action_offset=np.asarray(act["offset"], dtype=float),
            joint_pos_default=np.asarray(obs["joint_pos_offset_default"], dtype=float),
            obs_term_order=list(obs["term_order"]),
            obs_total_dim=int(obs["total_dim"]),
            control_hz=float(d.get("control_hz", 50.0)),
            gains=dict(d.get("gains", {})),
        )

    @property
    def n(self) -> int:
        return len(self.action_joint_names)


# ── live robot state ─────────────────────────────────────────────────────────
@dataclass
class RobotState:
    q: dict                  # joint-name -> position (rad)
    dq: dict                 # joint-name -> velocity (rad/s)
    quat_wxyz: tuple         # base orientation
    gyro: tuple              # base angular velocity (body frame, rad/s)
    base_lin_vel: tuple      # base linear velocity (body frame, m/s)


def quat_rotate_inverse(q_wxyz, v) -> np.ndarray:
    """Rotate world vector ``v`` into the body frame given body quaternion (wxyz)."""
    w, x, y, z = q_wxyz
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])
    return R.T @ np.asarray(v, dtype=float)


# ── robot interface a concrete binding implements ────────────────────────────
class RobotIO(ABC):
    @abstractmethod
    def read_state(self) -> RobotState: ...

    @abstractmethod
    def publish_targets(self, targets: dict, gains: dict, weight: Optional[float] = None) -> None:
        """Command joint position targets (name->q) with per-joint (kp,kd). ``weight`` drives an
        overlay blend where supported (e.g. G1 ``rt/arm_sdk`` ``motor_cmd[29].q``); None = full."""

    @abstractmethod
    def damp_once(self) -> None:
        """Issue ONE compliant command (kp=0, kd small, tau=0) across all joints. Non-blocking."""

    @abstractmethod
    def abort_tripped(self) -> bool:
        """True once the hardware controller abort has latched."""

    def arm_abort(self, timeout_s: float = 8.0) -> bool:
        """Wait for the controller to arm (buttons released). Default: assume armed."""
        return True

    # whole-body only ---------------------------------------------------------
    def release_vendor(self) -> bool:
        """Release the vendor balance controller and RETURN True only if release is confirmed.
        Default: not supported (so run_whole refuses)."""
        return False

    def reengage_vendor(self) -> None:
        """Re-select the vendor controller (graceful recovery if still upright)."""


# ── observation builder (term-major, generic over the contract) ──────────────
class ObsBuilder:
    def __init__(self, contract: DeployContract, command_term: str = "hand_target") -> None:
        self.c = contract
        self.command_term = command_term

    def build(self, state: RobotState, command, last_action: np.ndarray):
        names = self.c.action_joint_names
        q = np.array([state.q[n] for n in names])
        dq = np.array([state.dq[n] for n in names])
        terms = {
            "base_lin_vel": np.asarray(state.base_lin_vel, float),
            "base_ang_vel": np.asarray(state.gyro, float),
            "projected_gravity": quat_rotate_inverse(state.quat_wxyz, [0.0, 0.0, -1.0]),
            self.command_term: np.asarray(command, float),
            "joint_pos": q - self.c.joint_pos_default,
            "joint_vel": dq,
            "actions": np.asarray(last_action, float),
        }
        parts = []
        for term in self.c.obs_term_order:
            if term not in terms:
                raise KeyError(f"obs term {term!r} not known to ObsBuilder "
                               f"(command_term={self.command_term!r})")
            parts.append(terms[term].ravel())
        obs = np.concatenate(parts).astype(np.float32)
        if obs.shape[0] != self.c.obs_total_dim:
            raise ValueError(f"obs dim {obs.shape[0]} != contract {self.c.obs_total_dim}")
        return obs, terms


# ── the deploy driver ────────────────────────────────────────────────────────
class PolicyDeploy:
    def __init__(self, contract: DeployContract, policy, io: RobotIO, *,
                 command_term: str = "hand_target") -> None:
        self.c = contract
        self.io = io
        self.obs = ObsBuilder(contract, command_term)
        self.last_action = np.zeros(contract.n)
        self.dt = 1.0 / contract.control_hz
        self.policy = self._load_policy(policy)

    @staticmethod
    def _load_policy(policy):
        if callable(policy):
            return policy                       # injected fn (tests / custom runtime)
        import torch                            # lazy: tests with a mock policy need no torch
        net = torch.jit.load(policy)
        net.eval()
        def _run(obs):
            import torch as _t
            with _t.inference_mode():
                return net(_t.from_numpy(obs).unsqueeze(0)).squeeze(0).numpy()
        return _run

    def infer(self, obs) -> np.ndarray:
        a = np.asarray(self.policy(obs), dtype=float)
        self.last_action = a
        return a

    def targets(self, action) -> dict:
        tq = self.c.action_offset + self.c.action_scale * np.asarray(action, float)
        return {n: float(tq[i]) for i, n in enumerate(self.c.action_joint_names)}

    def gains(self) -> dict:
        return dict(self.c.gains)

    # -- rung 0: offline, read-only -------------------------------------------
    def run_offline(self, command, steps: int = 3, log: Callable = print):
        log("[ladder:0 offline] read-only — NO commands")
        out = []
        for k in range(steps):
            state = self.io.read_state()
            obs, parts = self.obs.build(state, command, self.last_action)
            a = self.infer(obs)
            tq = self.targets(a)
            out.append((a, tq))
            if k == 0:
                g = parts["projected_gravity"]
                log(f"  obs_dim={obs.shape[0]} gravity={np.round(g,3).tolist()} "
                    f"|a|max={np.abs(a).max():.3f} finite={np.isfinite(a).all()}")
        return out

    # -- rung 1: partial, fall-safe (subset via overlay) ----------------------
    def run_partial(self, command, subset: list, seconds: float, *, vmax_rad_s: float = 1.0,
                    clamp: Optional[dict] = None, blend_s: float = 1.2, log: Callable = print):
        """Apply ONLY ``subset`` joint targets via the overlay (weight) path; the rest stay on the
        vendor controller, so the robot can't fall. Rate-limited + clamped + motion-blended + abort."""
        if not self.io.arm_abort():
            log("[ladder:1] abort not armed — refusing")
            return
        clamp = clamp or {}
        cmdq = {n: self.io.read_state().q[n] for n in subset}
        max_step = vmax_rad_s * self.dt
        n_in = max(1, int(blend_s / self.dt))
        with SafeStop(self.io.damp_once, name="partial"):
            for k in range(n_in):                       # blend the overlay weight in, holding pose
                if self.io.abort_tripped():
                    return log("[ladder:1] abort during blend-in")
                self.io.publish_targets(cmdq, self.gains(), weight=(k + 1) / n_in)
                _sleep(self.dt)
            t_end = _now() + seconds
            while _now() < t_end:
                if self.io.abort_tripped():
                    return log("[ladder:1] controller abort")
                state = self.io.read_state()
                obs, _ = self.obs.build(state, command, self.last_action)
                tq = self.targets(self.infer(obs))
                for n in subset:
                    goal = tq[n]
                    if n in clamp:
                        goal = max(clamp[n][0], min(clamp[n][1], goal))
                    cmdq[n] += max(-max_step, min(max_step, goal - cmdq[n]))   # rate-limited
                self.io.publish_targets(cmdq, self.gains(), weight=1.0)
                _sleep(self.dt)
        log("[ladder:1] complete (overlay released, damped)")

    # -- rung 2: full whole-body takeover -------------------------------------
    def run_whole(self, command, seconds: float, *, settle_s: float = 1.5, blend_s: float = 2.5,
                  log: Callable = print):
        """Full takeover: release the vendor controller, then settle(hold)→blend→policy via full
        joint targets. REQUIRES a SUPPORTED robot (gantry). Refuses if release can't be verified."""
        if not self.io.arm_abort():
            log("[ladder:2] abort not armed — refusing")
            return
        if not self.io.release_vendor():
            log("[ladder:2] vendor controller NOT confirmed released — refusing to take over")
            return
        names = self.c.action_joint_names
        hold = {n: self.io.read_state().q[n] for n in names}
        g = self.gains()
        with SafeStop(self.io.damp_once, name="whole"):
            t0 = _now()
            while True:
                t = _now() - t0
                if self.io.abort_tripped():
                    return log("[ladder:2] controller abort")
                state = self.io.read_state()
                obs, _ = self.obs.build(state, command, self.last_action)
                pol = self.targets(self.infer(obs))
                if t < settle_s:
                    tgt = hold
                elif t < settle_s + blend_s:
                    al = (t - settle_s) / blend_s
                    tgt = {n: (1 - al) * hold[n] + al * pol[n] for n in names}
                elif t < settle_s + blend_s + seconds:
                    tgt = pol
                else:
                    break
                self.io.publish_targets(tgt, g, weight=None)
                _sleep(self.dt)
        log("[ladder:2] complete (damped)")


# time hooks (overridable in tests so they don't actually sleep)
def _now():
    return __import__("time").monotonic()


def _sleep(dt):
    __import__("time").sleep(dt)


# ── mock robot for off-robot tests ───────────────────────────────────────────
class MockRobotIO(RobotIO):
    """Records published targets and fakes a fixed standing state. ``abort_after`` trips the abort
    after N publishes; ``can_release`` toggles the whole-body release guard."""

    def __init__(self, joint_names, default, *, abort_after=None, can_release=True):
        self.names = list(joint_names)
        self._q = {n: float(default[i]) for i, n in enumerate(self.names)}
        self.published = []
        self.damps = 0
        self.released = False
        self._abort_after = abort_after
        self._can_release = can_release
        self._pub_count = 0

    def read_state(self):
        return RobotState(q=dict(self._q), dq={n: 0.0 for n in self.names},
                          quat_wxyz=(1.0, 0.0, 0.0, 0.0), gyro=(0.0, 0.0, 0.0),
                          base_lin_vel=(0.0, 0.0, 0.0))

    def publish_targets(self, targets, gains, weight=None):
        self.published.append((dict(targets), weight))
        self._pub_count += 1
        for n, q in targets.items():            # mock: joints instantly reach the target
            self._q[n] = q

    def damp_once(self):
        self.damps += 1

    def abort_tripped(self):
        return self._abort_after is not None and self._pub_count >= self._abort_after

    def release_vendor(self):
        self.released = self._can_release
        return self._can_release


__all__ = [
    "DeployContract", "RobotState", "RobotIO", "ObsBuilder", "PolicyDeploy",
    "MockRobotIO", "quat_rotate_inverse",
]
