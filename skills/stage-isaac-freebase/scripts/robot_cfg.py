"""Build a free-base humanoid ArticulationCfg from a robot descriptor — with DOF reconciliation.

This is the bridge from a [robot descriptor](../../discover-robot/schema/robot_descriptor.schema.json)
to an Isaac Lab ``ArticulationCfg`` configured for a physically-valid free-base balance task:

  * **Free base + gravity ON** (no kinematic cheats). Pair with the mobile-base USD from
    ``make_mobile_usd.py`` so ``fix_root_link=False`` yields a true floating base.
  * **Deploy PD gains** from the descriptor's ``effectors.pd_gains`` — the stock *manipulation* gains
    (e.g. the G1 Inspire preset's waist kp 5000 / arms kp 3000) COLLAPSE a whole-body balance policy.
  * **Neutral pose** from the descriptor's ``effectors.default_pose`` (taken from the robot's own
    walking-policy default, so a reach policy's neutral matches the deploy stance).
  * **DOF reconciliation** — the heart of real-to-sim for a mismatched asset. The sim asset may have
    MORE DOF than the real robot (e.g. a 29-DOF G1 USD vs. a 23-DOF G1 EDU). The joints the real robot
    lacks (``sim_asset.sim_real_reconciliation.locked_sim_joints``) are put in a stiff "locked" actuator
    group that HOLDS THEM AT DEFAULT, and are EXCLUDED from the policy's action set (:func:`action_joints`)
    so the trained policy only commands DOF the hardware actually has → transfer-valid. For a sim==real
    robot the locked list is empty and the policy commands every joint.

Lifted + generalized from the eye-verified armwaheed/robots#2 ``rl/robot_cfg.py`` (Unitree G1). The G1
path is proven; the descriptor drives the gains/pose/action-set/reconciliation so the same builder
serves any humanoid whose Isaac asset preset you pass in as ``base_cfg``.
"""

from __future__ import annotations

import json
import os

SPAWN_Z_DEFAULT = 0.80  # pelvis height with bent knees that plants the feet without punching the floor


def load_descriptor(path: str) -> dict:
    with open(path) as f:
        return json.load(f)


def locked_joints(descriptor: dict) -> list[str]:
    """Sim joints present in the asset but ABSENT on the real robot — held at default, never commanded."""
    return list(descriptor["sim_asset"]["sim_real_reconciliation"]["locked_sim_joints"])


def action_joints(descriptor: dict) -> list[str]:
    """The joints the policy commands = the sim asset's joints MINUS the locked (absent) ones.

    Falls back to ``effectors.joint_order`` when ``sim_asset.joint_order`` is not given. This is what a
    23-DOF robot uses to train a 23-DOF-action policy on a 29-DOF asset, and what a 29-DOF robot uses to
    command all 29."""
    sim = descriptor["sim_asset"]
    order = sim.get("joint_order") or descriptor["effectors"]["joint_order"]
    locked = set(locked_joints(descriptor))
    return [j for j in order if j not in locked]


def _actuator_groups(descriptor: dict):
    """One ImplicitActuatorCfg per ``effectors.pd_gains`` group, plus a stiff 'locked' group that holds
    the reconciliation's locked joints at their default."""
    from isaaclab.actuators import ImplicitActuatorCfg

    groups = {}
    for i, g in enumerate(descriptor["effectors"]["pd_gains"]):
        groups[f"group_{i}"] = ImplicitActuatorCfg(
            joint_names_expr=[g["joints"]],
            stiffness=float(g["stiffness"]),
            damping=float(g["damping"]),
            armature=0.01,
            effort_limit_sim=300.0,
            velocity_limit_sim=100.0,
        )
    locked = locked_joints(descriptor)
    if locked:
        # Hold the absent-on-hardware joints rigidly at default (stiff PD), so they never move and the
        # robot's effective kinematics match the real reduced-DOF robot. They are also excluded from the
        # action set (see action_joints), so the policy can't command them.
        groups["locked"] = ImplicitActuatorCfg(
            joint_names_expr=list(locked),
            stiffness=500.0,
            damping=20.0,
            armature=0.01,
            effort_limit_sim=300.0,
            velocity_limit_sim=100.0,
        )
    return groups


def make_robot_cfg(descriptor: dict, base_cfg, prim_path: str = "{ENV_REGEX_NS}/Robot",
                   mobile_usd: str | None = None, spawn_z: float = SPAWN_Z_DEFAULT):
    """Return a free-base ArticulationCfg for the descriptor's robot.

    ``base_cfg`` is the robot's Isaac asset preset (e.g. ``isaaclab_assets.robots.unitree.G1_INSPIRE_FTP_CFG``)
    — it supplies the USD-structural sim properties; this function overrides the USD path, the free-base
    flags, the deploy PD gains, the neutral pose, and the locked-joint group from the descriptor. Pass
    ``mobile_usd`` = the output of ``make_mobile_usd.py`` (the world-pin-removed USD); if omitted, the
    descriptor's ``sim_asset.usd_uri`` is used (only correct if it is already free-base)."""
    cfg = base_cfg.copy()
    cfg.prim_path = prim_path

    usd = mobile_usd or descriptor["sim_asset"]["usd_uri"]
    cfg.spawn.usd_path = os.path.abspath(usd) if os.path.exists(usd) else usd

    # Free base + gravity on (physically-valid balance task; no kinematic cheats).
    cfg.spawn.rigid_props.disable_gravity = False
    cfg.spawn.articulation_props.fix_root_link = False
    cfg.spawn.activate_contact_sensors = True

    # Deploy gains + locked-DOF group from the descriptor (replaces the stock BODY manipulation gains).
    # Preserve the base preset's hand/finger actuator group — the descriptor models only body joints, so
    # without this the fingers would be left unactuated (floppy). The descriptor's body groups + the
    # locked group must partition the body joints exactly once (verified by rl/check_spawn + the
    # actuator-partition check); the preserved hand group covers the fingers, disjoint from the body.
    HAND_GROUP_KEYS = {"hands", "hand", "fingers", "gripper"}
    preserved = {k: v for k, v in dict(cfg.actuators).items() if k.lower() in HAND_GROUP_KEYS}
    cfg.actuators = {**preserved, **_actuator_groups(descriptor)}

    # Neutral pose from the descriptor (the walking-policy default → clean walk→reach handoff).
    cfg.init_state = cfg.init_state.replace(
        pos=(0.0, 0.0, spawn_z),
        rot=(1.0, 0.0, 0.0, 0.0),
        joint_pos=dict(descriptor["effectors"]["default_pose"]),
        joint_vel={".*": 0.0},
    )
    return cfg


def make_g1_cfg(descriptor: dict, prim_path: str = "{ENV_REGEX_NS}/Robot", mobile_usd: str | None = None):
    """Convenience for a Unitree G1 descriptor: pulls the Isaac G1 Inspire preset as the base_cfg.

    The proven armwaheed/robots#2 path. For the 23-DOF G1 EDU descriptor this locks the 6 sim joints the
    EDU lacks (waist roll/pitch, L/R wrist pitch/yaw); for the 29-DOF descriptor it locks nothing."""
    from isaaclab_assets.robots.unitree import G1_INSPIRE_FTP_CFG

    return make_robot_cfg(descriptor, G1_INSPIRE_FTP_CFG, prim_path=prim_path, mobile_usd=mobile_usd)
