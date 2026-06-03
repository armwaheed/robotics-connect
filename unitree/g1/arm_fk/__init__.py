"""arm_fk — pure-numpy forward kinematics for the Unitree G1 arms.

Public API:
    G1ArmFK()                       — load the bundled URDF.
    fk.forward(arm_q14)             — dict of link positions in torso frame.
    fk.forward_body_frame(arm_q14)  — same, offset so origin = d435 camera mount.
    fk.palm_xyz(arm_q14)            — {"L_palm", "R_palm"} in body frame.
    fk.benchmark(n)                 — seconds per call.
"""
from .arm_fk import (
    G1ArmFK,
    URDF_PATH,
    TORSO_LINK,
    CAMERA_LINK,
    LEFT_PALM_LINK,
    RIGHT_PALM_LINK,
    LEFT_ELBOW_LINK,
    RIGHT_ELBOW_LINK,
    ARM_JOINT_NAMES_14,
)

__all__ = [
    "G1ArmFK",
    "URDF_PATH",
    "TORSO_LINK",
    "CAMERA_LINK",
    "LEFT_PALM_LINK",
    "RIGHT_PALM_LINK",
    "LEFT_ELBOW_LINK",
    "RIGHT_ELBOW_LINK",
    "ARM_JOINT_NAMES_14",
]
