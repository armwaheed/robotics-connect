#!/usr/bin/env python3
"""
arm_fk — pure-numpy forward kinematics for the Unitree G1 arms.

This module knows where the end-effectors are in the same body frame that
`depth_camera_sight.pixel_to_body_xyz` reports the table plane in.  That
lets a controller reason about palm height geometrically — e.g. "palm
within N cm of the live camera-measured table plane" — instead of a
hand-tuned shoulder-pitch joint threshold that only works on the one robot
it was calibrated against.  The geometric test is immune to base-pose
drift and table height.

Dependencies: numpy and Python stdlib (xml.etree.ElementTree).  No ROS,
no pinocchio, no urchin, no mesh loaders.  The price of that simplicity
is we only support what G1 needs: revolute and fixed joints, no mimic
joints, no continuous joints, no prismatic joints.  The bundled URDF
(urdf/g1_body29_hand14.urdf) is Unitree's stock G1 model and uses only
those supported types for the arm chains.

Frame conventions
-----------------
Torso frame: URDF's `torso_link`.  +X forward, +Y left, +Z up, origin at
the waist-pitch joint centre.  All joint origins and axes in the URDF
are relative to this chain.

Body frame: identical axes to torso frame but origin translated to the
d435 camera mount (link `d435_link`, which the URDF bolts to torso_link
with a fixed xyz + rpy).  We ignore the d435's rpy for position queries
because we are only comparing Z-coordinates against camera-reported
Z-coordinates, and the rpy only rotates the camera's *optical* frame
within the body frame — it doesn't move the body-frame origin.  See
depth_camera_sight.DepthCameraSight.pixel_to_body_xyz for the matching
definition (origin at camera mount, +X forward, +Y left, +Z up).

14-DOF arm_q layout
-------------------
    0  left_shoulder_pitch
    1  left_shoulder_roll
    2  left_shoulder_yaw
    3  left_elbow
    4  left_wrist_roll        (a.k.a. "forearm roll")
    5  phantom (left_wrist_pitch — not present on 23-DOF G1, treated as 0)
    6  phantom (left_wrist_yaw  — not present on 23-DOF G1, treated as 0)
    7  right_shoulder_pitch
    8  right_shoulder_roll
    9  right_shoulder_yaw
   10  right_elbow
   11  right_wrist_roll
   12  phantom (right_wrist_pitch)
   13  phantom (right_wrist_yaw)

For 23-DOF G1 EDU units, slots 5, 6, 12, 13 are ignored — the SDK does
not publish or accept values for those joints, and the URDF chain in
those slots folds down to an identity rotation at the fixed offset of
the (non-existent) wrist link, leaving the palm at its nominal wrist
position.  The 29-DOF URDF then gives us the correct geometric offset
from wrist_roll to palm via its fixed xyz chain.
"""
from __future__ import annotations

import os
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np


URDF_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "urdf", "g1_body29_hand14.urdf")

TORSO_LINK       = "torso_link"
CAMERA_LINK      = "d435_link"
LEFT_PALM_LINK   = "left_hand_palm_link"
RIGHT_PALM_LINK  = "right_hand_palm_link"
LEFT_ELBOW_LINK  = "left_elbow_link"
RIGHT_ELBOW_LINK = "right_elbow_link"
LEFT_WRIST_LINK  = "left_wrist_roll_link"
RIGHT_WRIST_LINK = "right_wrist_roll_link"
LEFT_SHOULDER_LINK  = "left_shoulder_pitch_link"
RIGHT_SHOULDER_LINK = "right_shoulder_pitch_link"


# 14-slot arm_q index → URDF joint name.  Phantom wrist slots on 23-DOF
# hardware map to the 29-DOF URDF joints so the FK still walks the full
# chain; callers on 23-DOF simply pass 0.0 for those slots (see the
# REST_POSE / EXTEND_POSE selftest fixtures).
ARM_JOINT_NAMES_14: Tuple[str, ...] = (
    "left_shoulder_pitch_joint",   # 0
    "left_shoulder_roll_joint",    # 1
    "left_shoulder_yaw_joint",     # 2
    "left_elbow_joint",            # 3
    "left_wrist_roll_joint",       # 4 (the "forearm roll")
    "left_wrist_pitch_joint",      # 5 phantom on 23-DOF
    "left_wrist_yaw_joint",        # 6 phantom on 23-DOF
    "right_shoulder_pitch_joint",  # 7
    "right_shoulder_roll_joint",   # 8
    "right_shoulder_yaw_joint",    # 9
    "right_elbow_joint",           # 10
    "right_wrist_roll_joint",      # 11
    "right_wrist_pitch_joint",     # 12 phantom on 23-DOF
    "right_wrist_yaw_joint",       # 13 phantom on 23-DOF
)

# Friendly names for common end-effector links, used in the return dict
# of `forward` / `forward_body_frame`.
_FRIENDLY_LINKS: Dict[str, str] = {
    LEFT_SHOULDER_LINK:  "L_shoulder",
    LEFT_ELBOW_LINK:     "L_elbow",
    LEFT_WRIST_LINK:     "L_wrist",
    LEFT_PALM_LINK:      "L_palm",
    RIGHT_SHOULDER_LINK: "R_shoulder",
    RIGHT_ELBOW_LINK:    "R_elbow",
    RIGHT_WRIST_LINK:    "R_wrist",
    RIGHT_PALM_LINK:     "R_palm",
}


# ── URDF parsing ─────────────────────────────────────────────────────────────

@dataclass
class _Joint:
    name: str
    type: str
    parent: str
    child: str
    xyz: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    rpy: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    axis: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0]))
    # Pre-computed static rotation from parent → joint origin (3x3).
    R_origin: np.ndarray = field(default_factory=lambda: np.eye(3, dtype=np.float64))


def _parse_vec3(s: str) -> np.ndarray:
    return np.array([float(x) for x in s.split()], dtype=np.float64)


def _rpy_to_matrix(rpy: np.ndarray) -> np.ndarray:
    """URDF RPY (roll about X, pitch about Y, yaw about Z) — applied as
    extrinsic rotations X→Y→Z, i.e. R = Rz · Ry · Rx.  This is the
    convention used by every URDF tool in the Unitree ecosystem
    (urdfdom, urchin, pinocchio) and matches what the robot expects.
    """
    rx, ry, rz = float(rpy[0]), float(rpy[1]), float(rpy[2])
    cr, sr = np.cos(rx), np.sin(rx)
    cp, sp = np.cos(ry), np.sin(ry)
    cy, sy = np.cos(rz), np.sin(rz)
    return np.array([
        [cy * cp,  cy * sp * sr - sy * cr,  cy * sp * cr + sy * sr],
        [sy * cp,  sy * sp * sr + cy * cr,  sy * sp * cr - cy * sr],
        [-sp,      cp * sr,                 cp * cr              ],
    ], dtype=np.float64)


def _axis_angle_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues rotation about a unit (or near-unit) axis.  URDF axes
    are almost always already unit-length; we normalise defensively.
    """
    n = float(np.linalg.norm(axis))
    if n < 1e-12:
        return np.eye(3, dtype=np.float64)
    x, y, z = axis / n
    c, s = np.cos(angle), np.sin(angle)
    C = 1.0 - c
    return np.array([
        [c + x * x * C,      x * y * C - z * s,  x * z * C + y * s],
        [y * x * C + z * s,  c + y * y * C,      y * z * C - x * s],
        [z * x * C - y * s,  z * y * C + x * s,  c + z * z * C   ],
    ], dtype=np.float64)


def _parse_urdf(path: str) -> Dict[str, _Joint]:
    tree = ET.parse(path)
    root = tree.getroot()
    joints: Dict[str, _Joint] = {}
    for j in root.findall("joint"):
        name = j.attrib["name"]
        jtype = j.attrib["type"]
        parent_el = j.find("parent")
        child_el = j.find("child")
        if parent_el is None or child_el is None:
            continue
        origin_el = j.find("origin")
        xyz = np.zeros(3, dtype=np.float64)
        rpy = np.zeros(3, dtype=np.float64)
        if origin_el is not None:
            if "xyz" in origin_el.attrib:
                xyz = _parse_vec3(origin_el.attrib["xyz"])
            if "rpy" in origin_el.attrib:
                rpy = _parse_vec3(origin_el.attrib["rpy"])
        axis = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        axis_el = j.find("axis")
        if axis_el is not None and "xyz" in axis_el.attrib:
            axis = _parse_vec3(axis_el.attrib["xyz"])
        joints[name] = _Joint(
            name=name,
            type=jtype,
            parent=parent_el.attrib["link"],
            child=child_el.attrib["link"],
            xyz=xyz,
            rpy=rpy,
            axis=axis,
            R_origin=_rpy_to_matrix(rpy),
        )
    return joints


def _build_child_to_parent_joint(joints: Dict[str, _Joint]) -> Dict[str, _Joint]:
    """Each link has exactly one parent joint in a URDF tree (except root)."""
    m: Dict[str, _Joint] = {}
    for j in joints.values():
        if j.child in m:
            # Not fatal, but URDF trees should not re-use a child link.
            raise RuntimeError(
                f"URDF: link {j.child!r} has more than one parent joint: "
                f"{m[j.child].name!r} and {j.name!r}")
        m[j.child] = j
    return m


def _chain_to_root(link: str, child_to_parent: Dict[str, _Joint],
                   root: str) -> List[_Joint]:
    """Ordered list of joints root → link (parent-first)."""
    rev: List[_Joint] = []
    cur = link
    while cur != root:
        if cur not in child_to_parent:
            raise RuntimeError(
                f"URDF: no parent joint for link {cur!r} while walking "
                f"toward {root!r} — is {root!r} really an ancestor of "
                f"{link!r}?")
        j = child_to_parent[cur]
        rev.append(j)
        cur = j.parent
    return list(reversed(rev))


# ── Public class ─────────────────────────────────────────────────────────────

class G1ArmFK:
    """Forward kinematics for the G1 arms, rooted at `torso_link`.

    Thread-safe for read-only queries (internal state is only the parsed
    URDF, which is never mutated after __init__).
    """

    def __init__(self, urdf_path: str = URDF_PATH):
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(
                f"arm_fk: URDF not found at {urdf_path}.  The bundled "
                f"file should live at urdf/g1_body29_hand14.urdf inside "
                f"the arm_fk package.")
        self._urdf_path = urdf_path
        self._joints: Dict[str, _Joint] = _parse_urdf(urdf_path)
        self._child_to_parent: Dict[str, _Joint] = _build_child_to_parent_joint(self._joints)

        # Pre-compute the chain of joints (torso → each target link) once.
        # FK then just walks these precomputed lists.
        self._chains: Dict[str, List[_Joint]] = {}
        for link in list(_FRIENDLY_LINKS.keys()) + [CAMERA_LINK]:
            self._chains[link] = _chain_to_root(link, self._child_to_parent, TORSO_LINK)

        # Camera mount offset (torso frame).  Fixed joint → constant offset.
        cam_chain = self._chains[CAMERA_LINK]
        for j in cam_chain:
            if j.type not in ("fixed",):
                raise RuntimeError(
                    f"arm_fk: expected d435 chain to be all-fixed; got "
                    f"{j.type} on joint {j.name!r}.  URDF may have been "
                    f"updated in an incompatible way.")
        self._camera_offset_torso: np.ndarray = self._fk_chain(cam_chain, {})[0]

    # ── low-level helpers ────────────────────────────────────────────────

    def _fk_chain(self, chain: List[_Joint],
                  joint_values: Dict[str, float]
                  ) -> Tuple[np.ndarray, np.ndarray]:
        """Walk an ordered chain of joints, returning (pos, R) of the
        final child link in the chain root's frame (torso_link).
        """
        pos = np.zeros(3, dtype=np.float64)
        R = np.eye(3, dtype=np.float64)
        for j in chain:
            # Translate by joint origin (in parent link's frame), then
            # rotate by joint origin rpy, then rotate about joint axis by
            # the commanded value.
            pos = pos + R @ j.xyz
            R = R @ j.R_origin
            if j.type == "fixed":
                continue
            q = float(joint_values.get(j.name, 0.0))
            if j.type in ("revolute", "continuous"):
                R = R @ _axis_angle_matrix(j.axis, q)
            else:
                raise RuntimeError(
                    f"arm_fk: unsupported joint type {j.type!r} on "
                    f"{j.name!r}.  Only revolute/continuous/fixed are "
                    f"implemented.")
        return pos, R

    # ── public API ───────────────────────────────────────────────────────

    @staticmethod
    def arm_q_to_joint_values(arm_q: np.ndarray) -> Dict[str, float]:
        """Map a 14-slot arm_q to a name-keyed dict suitable for feeding
        the FK chain walker.  Slots 5, 6, 12, 13 are the phantom
        wrist-pitch/yaw joints on 23-DOF G1 — they come through as 0.0 and
        are passed through unchanged.
        """
        if arm_q.shape[0] < 14:
            raise ValueError(
                f"arm_fk: arm_q must have at least 14 elements; got "
                f"{arm_q.shape[0]}")
        return {name: float(arm_q[i]) for i, name in enumerate(ARM_JOINT_NAMES_14)}

    def forward(self, arm_q: np.ndarray) -> Dict[str, np.ndarray]:
        """Return a dict {friendly_name: xyz} for every arm link of
        interest, expressed in `torso_link` frame.
        """
        jv = self.arm_q_to_joint_values(np.asarray(arm_q))
        out: Dict[str, np.ndarray] = {}
        for link, friendly in _FRIENDLY_LINKS.items():
            pos, _ = self._fk_chain(self._chains[link], jv)
            out[friendly] = pos.astype(np.float32)
        return out

    def forward_body_frame(self, arm_q: np.ndarray) -> Dict[str, np.ndarray]:
        """Same as `forward`, but translated so the origin sits at the
        d435 camera mount.  This is the frame used by
        `depth_camera_sight.pixel_to_body_xyz`, so values from this
        function can be compared directly against the camera's
        table-plane estimate.
        """
        torso = self.forward(arm_q)
        offset = self._camera_offset_torso.astype(np.float32)
        return {name: (xyz - offset) for name, xyz in torso.items()}

    def palm_xyz(self, arm_q: np.ndarray) -> Dict[str, np.ndarray]:
        """Convenience shortcut: body-frame {L_palm, R_palm} only.  This
        is the hot path used by the palm-over-table reach check.
        """
        all_body = self.forward_body_frame(arm_q)
        return {"L_palm": all_body["L_palm"], "R_palm": all_body["R_palm"]}

    @property
    def camera_offset_torso(self) -> np.ndarray:
        """Fixed xyz offset from torso_link origin to the d435 camera
        mount, in torso frame.  This equals
        `depth_camera_sight`'s body-frame origin expressed in torso coords.
        """
        return self._camera_offset_torso.copy()

    def benchmark(self, n: int = 1000,
                  arm_q: Optional[np.ndarray] = None) -> Dict[str, float]:
        """Time `palm_xyz` over `n` calls.  Returns a dict with
        total_s, per_call_us, and hz.
        """
        if arm_q is None:
            arm_q = np.zeros(14, dtype=np.float32)
        # Warm the numpy ufuncs.
        for _ in range(5):
            self.palm_xyz(arm_q)
        t0 = time.monotonic()
        for _ in range(n):
            self.palm_xyz(arm_q)
        dt = time.monotonic() - t0
        return {
            "total_s": dt,
            "per_call_us": 1e6 * dt / max(n, 1),
            "hz": n / max(dt, 1e-9),
        }


# ── Self-test / CLI ──────────────────────────────────────────────────────────

def _selftest() -> int:
    """Load the bundled URDF, run FK at two reference poses, and print
    palm positions in both torso and body frames.  Exits non-zero if any
    sanity check fails.
    """
    fk = G1ArmFK()
    print(f"arm_fk: loaded URDF from {fk._urdf_path}")
    print(f"        camera offset (torso frame): "
          f"{fk._camera_offset_torso.tolist()}")

    # Two reference arm poses used as selftest fixtures: a wide "rest"
    # pose (shoulders rolled out) and a forward "extend" pose.
    rest = np.array([
        -0.45294,  1.68574,  0.74384,  0.13688,  0.11514, 0.0, 0.0,
        -0.45294, -1.68574, -0.74384,  0.13688,  0.11514, 0.0, 0.0,
    ], dtype=np.float32)
    extend = np.array([
        -0.52465,  1.21007, -0.09538,  0.33268, -0.36082, 0.0, 0.0,
        -0.52465, -1.21007, -0.09538,  0.33268, -0.36082, 0.0, 0.0,
    ], dtype=np.float32)

    ok = True
    for name, q in (("REST_POSE", rest), ("EXTEND_POSE", extend)):
        torso = fk.forward(q)
        body = fk.forward_body_frame(q)
        print(f"\n{name}")
        print(f"  L_palm torso: {torso['L_palm'].tolist()}")
        print(f"  R_palm torso: {torso['R_palm'].tolist()}")
        print(f"  L_palm body:  {body['L_palm'].tolist()}")
        print(f"  R_palm body:  {body['R_palm'].tolist()}")

        # Determinism check: two calls with the same input must match bit-for-bit.
        t2 = fk.forward(q)
        for k in torso:
            if not np.array_equal(torso[k], t2[k]):
                print(f"  FAIL: determinism broken on {k}")
                ok = False

        # Sanity checks.
        l_body = body["L_palm"]
        r_body = body["R_palm"]
        if not (l_body[1] > 0 > r_body[1]):
            print(f"  FAIL: L_palm.y should be > 0 (left of camera) and "
                  f"R_palm.y should be < 0; got L.y={l_body[1]:.3f} "
                  f"R.y={r_body[1]:.3f}")
            ok = False
        # No blanket "palm below camera" rule: REST is the ape-hanger
        # pose with shoulders rolled wide and up, so the palms can sit at or
        # above the camera mount.  EXTEND has arms horizontal at shoulder
        # height and is checked below against a specific Z window.
        for k in ("L_palm", "R_palm", "L_elbow", "R_elbow"):
            if not np.all(np.isfinite(body[k])):
                print(f"  FAIL: non-finite position for {k}: {body[k]}")
                ok = False

    # EXTEND_POSE specifically reaches forward — palm_x should be
    # strictly greater in EXTEND than in REST (which has shoulders
    # rolled out wide to the sides rather than reaching forward).
    rest_body = fk.forward_body_frame(rest)
    ext_body  = fk.forward_body_frame(extend)
    if not (ext_body["L_palm"][0] > rest_body["L_palm"][0]):
        print(f"  FAIL: EXTEND L_palm.x ({ext_body['L_palm'][0]:.3f}) "
              f"should exceed REST L_palm.x "
              f"({rest_body['L_palm'][0]:.3f})")
        ok = False
    if not (ext_body["R_palm"][0] > rest_body["R_palm"][0]):
        print(f"  FAIL: EXTEND R_palm.x should exceed REST R_palm.x")
        ok = False
    # EXTEND places the arms at shoulder height (L_SHPITCH ≈ -0.525,
    # elbow ≈ 0.33) — palms should sit well below the camera (which is
    # ~18 cm above the shoulders on the head) but above the nominal
    # table plane (~50 cm below camera on the calibrated unit).  This
    # is the critical invariant the REACH handoff trigger relies on,
    # so we check it precisely here.
    for k in ("L_palm", "R_palm"):
        z = float(ext_body[k][2])
        if not (-0.45 < z < -0.05):
            print(f"  FAIL: EXTEND {k}.z={z:.3f} outside expected "
                  f"window (-0.45, -0.05) — FK may be mis-walking "
                  f"the wrist sub-chain")
            ok = False

    stats = fk.benchmark(n=500)
    print(f"\nBenchmark: {stats['per_call_us']:.1f} us/call "
          f"({stats['hz']:.0f} Hz)")

    if ok:
        print("\nSELFTEST_OK")
        return 0
    print("\nSELFTEST_FAILED")
    return 1


if __name__ == "__main__":
    import sys
    sys.exit(_selftest())
