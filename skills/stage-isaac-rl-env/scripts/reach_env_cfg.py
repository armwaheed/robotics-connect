"""Whole-body reach / loco-manipulation RL env template (manager-based Isaac Lab).

A free-base humanoid learns to BALANCE on its own feet while reaching a commanded hand target in a
forward+down workspace — the loco-manipulation skill a pure walking policy can't hold (a deep lean
throws the CoM past the feet). Physically valid for sim-to-real: NO base pinning / teleporting / joint
freezing. The legs balance the free base through the normal actuator path; the waist + arms reach.

Generalized from the eye-verified armwaheed/robots#2 `rl/bed_reach_env_cfg.py`. The robot, the **action
joint set**, and the **EE links** are read from a [robot descriptor](../../discover-robot/SKILL.md) via
`stage-isaac-freebase/robot_cfg.py`, so a 23-DOF robot trains a 23-DOF-action policy and a 29-DOF robot
commands all 29 — same env. Set the descriptor and the free-base USD with env vars:

    ROBOTICS_CONNECT_DESCRIPTOR=.../discover-robot/descriptors/unitree_g1_edu.json
    ROBOTICS_CONNECT_MOBILE_USD=.../stage-isaac-freebase/assets/g1_inspire_mobile.usd

The reach target is sampled in the robot's BASE frame (UniformPoseCommand), so the policy is
yaw-invariant; the reward tracks whichever hand is on the target's side (ambidextrous, see mdp.py). A
task-surface obstacle (the bed) sits in front so the robot must bend OVER it — the real bedside
constraint a free-space policy never feels (and why it walks off its spot without station-keeping).
"""

from __future__ import annotations

import os
import sys

import isaaclab.envs.mdp as base_mdp
import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import CameraCfg, ContactSensorCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise
from isaaclab_tasks.manager_based.locomotion.velocity.mdp import feet_slide as feet_slide_fn

from mdp import (
    base_xy_anchor_l2,
    idle_arm_deviation_l1,
    randomize_ee_load,
    same_side_position_error,
    same_side_position_error_tanh,
)

# Pull the descriptor-driven robot cfg from the stage-isaac-freebase skill (co-located in the plugin).
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, os.pardir,
                                "stage-isaac-freebase", "scripts"))
import robot_cfg as rc  # noqa: E402

DESCRIPTOR_PATH = os.environ.get(
    "ROBOTICS_CONNECT_DESCRIPTOR",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, os.pardir,
                 "discover-robot", "descriptors", "unitree_g1_29dof.json"),
)
MOBILE_USD = os.environ.get("ROBOTICS_CONNECT_MOBILE_USD", None)

DESCRIPTOR = rc.load_descriptor(DESCRIPTOR_PATH)
ACTION_JOINTS = rc.action_joints(DESCRIPTOR)                       # excludes locked (absent-on-hw) joints
_EE = DESCRIPTOR["effectors"].get("ee_links", {})
RIGHT_EE_BODY = _EE.get("right", "right_wrist_yaw_link")
LEFT_EE_BODY = _EE.get("left", "left_wrist_yaw_link")
PELVIS_BODY = "pelvis"
FOOT_BODIES = ".*_ankle_roll_link"
WAIST_JOINTS = [j["present_joints"] for j in DESCRIPTOR["effectors"]["morphology"] if j["segment"] == "waist"]
WAIST_JOINTS = WAIST_JOINTS[0] if WAIST_JOINTS else ["waist_yaw_joint"]

# Both hands in [right, left] order (preserve_order so the ambidextrous mdp terms see body_ids[0]=right,
# body_ids[1]=left). The reach is solved with whichever hand is on the target's side.
HANDS = SceneEntityCfg("robot", body_names=[RIGHT_EE_BODY, LEFT_EE_BODY], preserve_order=True)


@configclass
class ReachSceneCfg(InteractiveSceneCfg):
    """Flat ground + the free-base humanoid + a task-surface obstacle (a bed) in front. The robot faces
    +x (small yaw noise) and the surface sits just ahead, so it must bend OVER it — feet outside, knees
    against it — to reach: the real constraint a free-space reach never feels. The surface is a static
    collision obstacle, NOT terminated on contact (only a fall ends the episode), so the policy learns to
    work against it, not avoid all contact."""

    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply", restitution_combine_mode="multiply",
            static_friction=1.0, dynamic_friction=1.0,
        ),
        debug_vis=False,
    )

    robot = rc.make_g1_cfg(DESCRIPTOR, prim_path="{ENV_REGEX_NS}/Robot", mobile_usd=MOBILE_USD)

    task_surface = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Bed",
        spawn=sim_utils.CuboidCfg(
            size=(1.4, 2.4, 0.66),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.6, dynamic_friction=0.6, restitution=0.0),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.42, 0.30, 0.22)),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.90, 0.0, 0.33)),
    )

    contact_forces = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/.*", history_length=3, track_air_time=True)

    eval_cam: CameraCfg | None = None  # populated in _PLAY

    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(
            intensity=750.0,
            texture_file=f"{ISAAC_NUCLEUS_DIR}/Materials/Textures/Skies/PolyHaven/kloofendal_43d_clear_puresky_4k.hdr",
        ),
    )


@configclass
class CommandsCfg:
    """A hand-target pose command sampled in the robot's base frame (position-dominant). pos_y spanning
    both sides is what makes the reach ambidextrous — a +y target is reached with the LEFT hand, a −y
    target with the RIGHT (see HANDS), so a sideways drag is a natural abduction for whichever hand leads."""

    hand_target = base_mdp.UniformPoseCommandCfg(
        asset_name="robot",
        body_name=RIGHT_EE_BODY,
        resampling_time_range=(3.0, 5.0),
        debug_vis=True,
        ranges=base_mdp.UniformPoseCommandCfg.Ranges(
            pos_x=(0.18, 0.55), pos_y=(-0.40, 0.40), pos_z=(-0.16, 0.10),
            roll=(0.0, 0.0), pitch=(0.0, 0.0), yaw=(0.0, 0.0),
        ),
    )


@configclass
class ActionsCfg:
    """Position targets on the descriptor's ACTION joints (the joints the real robot has — locked
    sim-only joints are excluded by stage-isaac-freebase.action_joints). Hand fingers are not part of
    the policy — they stay at default."""

    joint_pos = base_mdp.JointPositionActionCfg(
        asset_name="robot", joint_names=ACTION_JOINTS, scale=0.5, use_default_offset=True
    )


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        base_lin_vel = ObsTerm(func=base_mdp.base_lin_vel, noise=Unoise(n_min=-0.1, n_max=0.1))
        base_ang_vel = ObsTerm(func=base_mdp.base_ang_vel, noise=Unoise(n_min=-0.2, n_max=0.2))
        projected_gravity = ObsTerm(func=base_mdp.projected_gravity, noise=Unoise(n_min=-0.05, n_max=0.05))
        hand_target = ObsTerm(func=base_mdp.generated_commands, params={"command_name": "hand_target"})
        joint_pos = ObsTerm(func=base_mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        joint_vel = ObsTerm(func=base_mdp.joint_vel_rel, noise=Unoise(n_min=-1.5, n_max=1.5))
        actions = ObsTerm(func=base_mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    physics_material = EventTerm(
        func=base_mdp.randomize_rigid_body_material,
        mode="startup",
        params={"asset_cfg": SceneEntityCfg("robot", body_names=".*"),
                "static_friction_range": (0.7, 1.1), "dynamic_friction_range": (0.5, 0.9),
                "restitution_range": (0.0, 0.0), "num_buckets": 64},
    )
    reset_base = EventTerm(
        func=base_mdp.reset_root_state_uniform,
        mode="reset",
        # No xy spawn offset: the base spawns AT the env origin = the station-keeping anchor. Yaw noise
        # is small (±15°) because the surface is a fixed obstacle in front (+x): the robot must face it.
        params={"pose_range": {"x": (0.0, 0.0), "y": (0.0, 0.0), "yaw": (-0.26, 0.26)},
                "velocity_range": {"x": (-0.2, 0.2), "y": (-0.2, 0.2), "z": (-0.1, 0.1),
                                   "roll": (-0.2, 0.2), "pitch": (-0.2, 0.2), "yaw": (-0.2, 0.2)}},
    )
    reset_robot_joints = EventTerm(
        func=base_mdp.reset_joints_by_scale, mode="reset",
        params={"position_range": (0.9, 1.1), "velocity_range": (0.0, 0.0)},
    )
    push_robot = EventTerm(
        func=base_mdp.push_by_setting_velocity, mode="interval", interval_range_s=(4.0, 7.0),
        params={"velocity_range": {"x": (-0.3, 0.3), "y": (-0.3, 0.3)}},
    )
    # Grip-slip / sheet-tension load on the ACTIVE hand: a random horizontal force toggling on/off every
    # 1-2.5 s — sudden load changes the policy must absorb (FALCON-style force-adaptive WBC).
    ee_load = EventTerm(
        func=randomize_ee_load, mode="interval", interval_range_s=(1.0, 2.5),
        params={"command_name": "hand_target", "asset_cfg": HANDS, "force_range": (0.0, 35.0), "slip_prob": 0.4},
    )


@configclass
class RewardsCfg:
    # -- task: reach the SAME-SIDE hand to the target (coarse shaping + sharp bonus near it)
    reach_coarse = RewTerm(func=same_side_position_error_tanh, weight=2.0,
                           params={"std": 0.20, "command_name": "hand_target", "asset_cfg": HANDS})
    reach_fine = RewTerm(func=same_side_position_error_tanh, weight=1.5,
                         params={"std": 0.06, "command_name": "hand_target", "asset_cfg": HANDS})
    reach_l2 = RewTerm(func=same_side_position_error, weight=-0.3,
                       params={"command_name": "hand_target", "asset_cfg": HANDS})

    # -- balance / staying alive (the hard part: hold balance through the lean)
    termination_penalty = RewTerm(func=base_mdp.is_terminated, weight=-200.0)
    upright = RewTerm(func=base_mdp.flat_orientation_l2, weight=-1.0)
    # Stay PLANTED: penalize the base drifting (xy) off its spawn spot — THE fix for the deploy topple.
    # Quadratic, so a small lean (~0.1 m) is nearly free while a backward step (~0.5-1 m) is heavily
    # penalized: the policy learns to reach + drag by leaning with its feet planted.
    base_anchor = RewTerm(func=base_xy_anchor_l2, weight=-2.0,
                          params={"asset_cfg": SceneEntityCfg("robot", body_names=PELVIS_BODY)})
    base_height = RewTerm(func=base_mdp.base_height_l2, weight=-0.5,
                          params={"target_height": 0.70, "asset_cfg": SceneEntityCfg("robot", body_names=PELVIS_BODY)})
    feet_slide = RewTerm(func=feet_slide_fn, weight=-0.2,
                         params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=FOOT_BODIES),
                                 "asset_cfg": SceneEntityCfg("robot", body_names=FOOT_BODIES)})

    # -- regularizers (smooth, sim-to-real-able motion)
    action_rate_l2 = RewTerm(func=base_mdp.action_rate_l2, weight=-0.01)
    dof_acc_l2 = RewTerm(func=base_mdp.joint_acc_l2, weight=-2.5e-7)
    dof_torques_l2 = RewTerm(func=base_mdp.joint_torques_l2, weight=-1.0e-5)
    dof_pos_limits = RewTerm(func=base_mdp.joint_pos_limits, weight=-1.0,
                             params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_ankle_.*_joint", ".*_knee_joint"])})
    joint_deviation_hips = RewTerm(func=base_mdp.joint_deviation_l1, weight=-0.15,
                                   params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_hip_yaw_joint", ".*_hip_roll_joint"])})
    joint_deviation_waist = RewTerm(func=base_mdp.joint_deviation_l1, weight=-0.05,
                                    params={"asset_cfg": SceneEntityCfg("robot", joint_names=WAIST_JOINTS)})
    # Keep the IDLE arm natural (penalize it leaving its at-side default); the active arm is free.
    idle_arm = RewTerm(
        func=idle_arm_deviation_l1, weight=-0.2,
        params={"command_name": "hand_target",
                "right_arm_cfg": SceneEntityCfg("robot", joint_names=["right_shoulder_.*_joint", "right_elbow_joint", "right_wrist_.*_joint"]),
                "left_arm_cfg": SceneEntityCfg("robot", joint_names=["left_shoulder_.*_joint", "left_elbow_joint", "left_wrist_.*_joint"])},
    )


@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=base_mdp.time_out, time_out=True)
    fell_over = DoneTerm(func=base_mdp.bad_orientation, params={"limit_angle": 1.0})  # ~57° pelvis tilt
    torso_contact = DoneTerm(func=base_mdp.illegal_contact,
                             params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=["pelvis", "torso_link"]), "threshold": 1.0})


@configclass
class ReachEnvCfg(ManagerBasedRLEnvCfg):
    scene: ReachSceneCfg = ReachSceneCfg(num_envs=2048, env_spacing=2.5)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()

    def __post_init__(self):
        self.decimation = 4
        self.episode_length_s = 8.0
        self.sim.dt = 0.005  # 200 Hz physics, 50 Hz control
        self.sim.render_interval = self.decimation
        self.sim.physx.gpu_max_rigid_patch_count = 10 * 2**15
        if self.scene.contact_forces is not None:
            self.scene.contact_forces.update_period = self.sim.dt


@configclass
class ReachEnvCfg_PLAY(ReachEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 16
        self.scene.env_spacing = 3.0
        self.observations.policy.enable_corruption = False
        self.events.push_robot = None
        self.commands.hand_target.resampling_time_range = (4.0, 4.0)  # hold a fixed target — read by eye
        self.scene.eval_cam = CameraCfg(
            prim_path="{ENV_REGEX_NS}/eval_cam", update_period=0, height=720, width=1280, data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(focal_length=22.0, focus_distance=400.0, horizontal_aperture=20.955, clipping_range=(0.05, 1.0e5)),
            offset=CameraCfg.OffsetCfg(pos=(2.4, 2.4, 1.7), rot=(1.0, 0.0, 0.0, 0.0), convention="world"),
        )
