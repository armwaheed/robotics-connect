"""Deploy a trained policy OUTSIDE its RL env — in a control loop / multi-robot demo.

Two reusable pieces, lifted from the eye-verified armwaheed/robots#2 `locomotion.py`:

1. :func:`load_onnx_mlp_to_torch` — lift an exported MLP-actor ONNX (a Gemm/ELU stack) into a torch
   ``Sequential`` on the GPU. The DGX Spark (aarch64 / GB10) has **no onnxruntime CUDA execution
   provider**, and the project rule is no CPU inference, so we run the policy through torch.cuda
   instead. ONNX Gemm ``transB=1`` stores weight as (out, in) — exactly ``torch.nn.Linear`` layout, so
   the weights map 1:1 (parity ~3e-6).

2. :class:`ReachPolicy` — run a trained whole-body reach policy out of the RL harness by reconstructing
   its EXACT observation and action map (the obs term order, ``action = raw*scale + default``, the joint
   order). The policy owns ALL action joints (legs balance, waist + arms reach) in a single forward pass,
   so this is the only control call per step. The reach is ambidextrous — it reaches with whichever hand
   is on the target's side — with NO observation change (the policy already sees the target).

:class:`VelocityWalker` runs Unitree's velocity-walk policy the same way (ONNX→torch), to walk the robot
to the work site before the reach policy takes over (the behaviour layer owns the walk→reach handoff).

Generalized so the joint set, EE links, and action scale come from a robot descriptor.
"""

from __future__ import annotations

import json
from collections import deque


def load_descriptor(path: str) -> dict:
    with open(path) as f:
        return json.load(f)


def load_onnx_mlp_to_torch(onnx_path: str, device: str):
    """Lift an exported MLP-actor ONNX (Gemm/ELU stack) into a torch ``Sequential`` on ``device``.

    No onnxruntime GPU provider on aarch64/GB10 → run the MLP through torch.cuda. Weights map 1:1: ONNX
    Gemm ``transB=1`` stores weight as (out, in), exactly ``torch.nn.Linear`` layout."""
    import onnx
    import torch
    from onnx import numpy_helper

    init = {w.name: numpy_helper.to_array(w) for w in onnx.load(onnx_path).graph.initializer}
    pairs = [("actor.0.weight", "actor.0.bias"), ("actor.2.weight", "actor.2.bias"),
             ("actor.4.weight", "actor.4.bias"), ("actor.6.weight", "actor.6.bias")]
    layers = []
    for k, (wn, bn) in enumerate(pairs):
        W = torch.tensor(init[wn], dtype=torch.float32, device=device)  # (out, in)
        b = torch.tensor(init[bn], dtype=torch.float32, device=device)
        lin = torch.nn.Linear(W.shape[1], W.shape[0]).to(device)
        with torch.no_grad():
            lin.weight.copy_(W)
            lin.bias.copy_(b)
        layers.append(lin)
        if k < len(pairs) - 1:
            layers.append(torch.nn.ELU(alpha=1.0))
    return torch.nn.Sequential(*layers).to(device).eval()


# ── Velocity-walk deploy (Unitree unitree_rl_lab, Isaac-Lab-native) ──────────────
# Spec verbatim from the policy's deploy runtime (see a descriptor's walk_policy):
#   obs = 5-step term-major history of [ang_vel*0.2 (3), proj_gravity (3), velocity_commands (3),
#         joint_pos_rel (29), joint_vel_rel*0.05 (29), last_action (29)] = 480
#   action = raw*0.25 + default_joint_pos      (all 29 body joints, Isaac articulation order)
VEL_ANG_VEL_SCALE = 0.2
VEL_DOF_VEL_SCALE = 0.05
VEL_ACTION_SCALE = 0.25
VEL_HISTORY = 5
VEL_CMD_RANGES = ((-0.5, 1.0), (-0.3, 0.3), (-0.2, 0.2))  # vx, vy, wz


class VelocityWalker:
    """Drive one robot with the velocity-walk policy (whole-body, 29-joint). At a non-zero command it
    strides to track ``[vx, vy, wz]``; at zero command it balances in place. The MLP runs on the GPU via
    torch (no onnxruntime). The 480-D obs, 5-step term-major history, ``*0.25 + default`` action and the
    Isaac joint order are reproduced from the policy's deploy spec. Pass the joint order + default pose
    from the descriptor's ``sim_asset.joint_order`` / ``effectors.default_pose``."""

    def __init__(self, robot, device: str, onnx_path: str, joint_names: list[str], default_pos: list[float]):
        import torch

        self._torch = torch
        self.robot = robot
        self.device = device
        self.policy = load_onnx_mlp_to_torch(onnx_path, device)
        self.joint_ids, _ = robot.find_joints(joint_names, preserve_order=True)
        self.default = torch.tensor(default_pos, device=device)
        self.last_action = torch.zeros(len(joint_names), device=device)
        self._hist = None

    def reset(self) -> None:
        self.last_action = self._torch.zeros(len(self.joint_ids), device=self.device)
        self._hist = None

    def _terms(self, cmd):
        torch = self._torch
        d = self.robot.data
        ang = d.root_ang_vel_b[0] * VEL_ANG_VEL_SCALE
        grav = d.projected_gravity_b[0]
        cmd_t = torch.tensor(cmd, device=self.device, dtype=torch.float32)
        q = d.joint_pos[0, self.joint_ids]
        jpos = q - self.default
        jvel = d.joint_vel[0, self.joint_ids] * VEL_DOF_VEL_SCALE
        return [ang, grav, cmd_t, jpos, jvel, self.last_action]

    def act(self, cmd) -> None:
        """Run the policy for ``[vx, vy, wz]`` and set joint targets. Call once per control step (~50 Hz)."""
        terms = self._terms(cmd)
        if self._hist is None:
            self._hist = [deque([t.clone() for _ in range(VEL_HISTORY)], maxlen=VEL_HISTORY) for t in terms]
        else:
            for k, t in enumerate(terms):
                self._hist[k].append(t)
        obs = self._torch.cat([h for term_hist in self._hist for h in term_hist]).unsqueeze(0)
        raw = self.policy(obs.float()).detach()[0]
        self.last_action = raw
        tgt = raw * VEL_ACTION_SCALE + self.default
        self.robot.set_joint_position_target(tgt.unsqueeze(0), joint_ids=self.joint_ids)


class ReachPolicy:
    """Drive one robot with a whole-body reach policy OUT of its RL env: reach a commanded WORLD hand
    target while balancing on its own feet (no pin/teleport/freeze), with whichever hand is on the
    target's side (ambidextrous). Call :meth:`set_world_target` to aim it, then :meth:`act` once per
    control step (~50 Hz). The exported actor is a stateless MLP, so the only per-robot state is the
    ``last_action`` history term — one instance per robot.

    The observation is rebuilt to MATCH THE TRAINING ENV EXACTLY (reach_env_cfg.ObservationsCfg):
        base_lin_vel(3) base_ang_vel(3) projected_gravity(3) hand_target(7 = base-frame pos+quat)
        joint_pos_rel(ALL joints) joint_vel_rel(ALL joints) last_action(len action_joints)
    Action = raw * action_scale + default_joint_pos, applied to the descriptor's action joints in
    articulation order. The hand target is given in the BASE frame (so the policy is yaw-invariant); we
    clamp it into the trained command box so a receding base can't drive an out-of-distribution command
    (which the policy chases by leaning harder → steps back → runaway → topple)."""

    # The base-frame command box the policy was trained on (reach_env_cfg CommandsCfg), pulled in
    # slightly for deploy margin so a fixed world target never lands outside the trained reach cone.
    CMD_CLAMP = ((0.20, 0.53), (-0.38, 0.38), (-0.14, 0.08))  # (x),(y),(z) base frame

    def __init__(self, robot, device: str, policy_path: str, descriptor: dict, action_scale: float = 0.5):
        import torch

        self._torch = torch
        self.robot = robot
        self.device = device
        self.action_scale = action_scale
        self.policy = torch.jit.load(policy_path, map_location=device)
        self.policy.eval()
        # Action joints = sim joints minus the locked (absent-on-hardware) ones — see stage-isaac-freebase.
        sim = descriptor["sim_asset"]
        order = sim.get("joint_order") or descriptor["effectors"]["joint_order"]
        locked = set(sim["sim_real_reconciliation"]["locked_sim_joints"])
        self.action_joint_names = [j for j in order if j not in locked]
        self.body_ids, _ = robot.find_joints(self.action_joint_names, preserve_order=False)
        ee = descriptor["effectors"].get("ee_links", {})
        rid, _ = robot.find_bodies([ee.get("right", "right_wrist_yaw_link")])
        lid, _ = robot.find_bodies([ee.get("left", "left_wrist_yaw_link")])
        self.right_ee_id, self.left_ee_id = int(rid[0]), int(lid[0])
        self.last_action = torch.zeros(len(self.body_ids), device=device)
        self._target_w = None
        self._cmd_quat = torch.tensor([1.0, 0.0, 0.0, 0.0], device=device)  # identity (training fixed rpy=0)

    def reset(self) -> None:
        self.last_action = self._torch.zeros(len(self.body_ids), device=self.device)

    def set_world_target(self, xyz) -> None:
        """Aim the reaching hand at a WORLD point (x, y, z). Transformed into the base frame each step
        (so it stays correct as the robot leans/turns); the policy reaches with whichever hand is on the
        target's side — aim it headward and each flanking robot uses its natural hand."""
        self._target_w = self._torch.tensor(xyz, device=self.device, dtype=self._torch.float32)

    def _hand_target_b(self):
        """The 7-dim base-frame command [pos_b(3), quat(4)] the obs term expects."""
        from isaaclab.utils.math import quat_rotate_inverse

        torch = self._torch
        d = self.robot.data
        root_p, root_q = d.root_pos_w[0], d.root_quat_w[0]
        tgt = self._target_w if self._target_w is not None else root_p
        pos_b = quat_rotate_inverse(root_q.unsqueeze(0), (tgt - root_p).unsqueeze(0))[0]
        (xlo, xhi), (ylo, yhi), (zlo, zhi) = self.CMD_CLAMP
        pos_b = torch.stack([pos_b[0].clamp(xlo, xhi), pos_b[1].clamp(ylo, yhi), pos_b[2].clamp(zlo, zhi)])
        return torch.cat([pos_b, self._cmd_quat])

    def _obs(self):
        torch = self._torch
        d = self.robot.data
        base_lin = d.root_lin_vel_b[0]                       # (3)
        base_ang = d.root_ang_vel_b[0]                       # (3)
        grav = d.projected_gravity_b[0]                      # (3)
        hand_t = self._hand_target_b()                       # (7)
        jpr = d.joint_pos[0] - d.default_joint_pos[0]        # ALL joints, articulation order
        jvr = d.joint_vel[0] - d.default_joint_vel[0]
        return torch.cat([base_lin, base_ang, grav, hand_t, jpr, jvr, self.last_action]).unsqueeze(0)

    def act(self) -> None:
        """Run the policy once and set the action-joint position targets. The policy owns the whole body,
        so this is the ONLY control call per step."""
        raw = self.policy(self._obs().float()).detach()[0]
        self.last_action = raw
        tgt = raw * self.action_scale + self.robot.data.default_joint_pos[0, self.body_ids]
        self.robot.set_joint_position_target(tgt.unsqueeze(0), joint_ids=self.body_ids)

    def _active_is_left(self) -> bool:
        """True when reaching with the LEFT hand — the commanded target is on the robot's left (base +y)."""
        return bool(self._hand_target_b()[1] >= 0.0)

    def active_wrist_link(self, descriptor: dict) -> str:
        """The wrist link the active (same-side) hand uses — where a cloth grasp attaches."""
        ee = descriptor["effectors"].get("ee_links", {})
        return ee.get("left") if self._active_is_left() else ee.get("right")
