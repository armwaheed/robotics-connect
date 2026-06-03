#!/usr/bin/env python3
"""
LiDAR Sight — singleton service for the G1 EDU's chest-mounted Livox MID-360.

Mid-field environmental perception via pure geometric
point-cloud analysis (RANSAC + height-band + clustering — NO neural network).
Sibling to Depth Camera Sight: the depth camera is angled downward and
only sees the near floor + near table; LiDAR fills the 1-5 m mid-field.

HARDWARE
--------
The LiDAR on this robot is a Livox MID-360 running on the root
`lidar_driver` service.  That service republishes the Livox SDK output as a
standard `sensor_msgs/msg/PointCloud2` over DDS on topic

    rt/utlidar/cloud_livox_mid360

at 10 Hz, ~20,000 points per frame.  Fields: x, y, z (float32 metres),
intensity (float32), ring (uint16), time (float32).  point_step = 22 bytes.
header.frame_id = "livox_frame" (== mid360_link).

We do NOT talk to the Livox SDK directly.  The root service already handles
the LiDAR's UDP 56000/56100 protocol and exposes the cleaned cloud over DDS,
so a user-space DDS subscriber is all we need — no extra conda env, no TCP
bridge, no second copy of the driver competing for the device.  This mirrors
the depth_camera_sight approach of using Unitree DDS for the RGB stream.

BODY FRAME
----------
All consumer-facing output is in torso body frame:

    +X forward, +Y left, +Z up, origin at torso_link.

The mid360_link → torso_link fixed transform is read once at import time
from the repo-bundled G1 URDF (`arm_fk/urdf/g1_body29_hand14.urdf`,
the same URDF `arm_fk` uses — keeping the two modules in sync), and falls
back to hard-coded constants if the file is missing.  The bundled URDF has:

    <joint name="mid360_joint" type="fixed">
      <origin xyz="0.0002835 0.00003 0.41618"
              rpy="0 0.04014257279586953 0"/>
      <parent link="torso_link"/>
      <child  link="mid360_link"/>
    </joint>

i.e. the LiDAR sits 41.6 cm above torso_link, essentially on the centreline,
with a 2.3° nose-down pitch.

BUT — the raw cloud on `rt/utlidar/cloud_livox_mid360` arrives in a frame
that is 180° rolled about +X relative to that URDF definition (verified
on 2026-04-17 against two ground-truth landmarks; see the
`_MOUNT_CORRECTION_ROLL_RAD` comment below).  Before the URDF transform
is applied we therefore add π to the URDF roll, effectively treating the
raw data as if it came from a LiDAR with `rpy="π 0.0401 0"`.  Arm_fk and
anyone else reading the URDF file directly see the untouched Unitree-
shipped document.

This transform is applied in the capture thread so consumers always see
body-frame data.  Raw livox-frame points are never exposed.

TABLE DETECTION ALGORITHM (v1)
------------------------------
A table, geometrically, is a horizontal plane at waist-to-shoulder height.
We do not need ML for that.  The pipeline:

1. Voxel-downsample the cloud to ~5 cm.
2. Estimate the FLOOR z by picking the lowest-but-populated bin of a coarse
   z histogram.  Points with z in (floor_z, floor_z + min_table_h_m] are
   dropped (they are floor noise, legs, or low obstacles — not tables).
3. Filter to the "table band": points with z in
   [floor_z + MIN_TABLE_H_M, floor_z + MAX_TABLE_H_M] (default 0.55-1.15 m).
4. Histogram the band-filtered z.  Each spike is a candidate table plane.
5. For each plane, take its z-inliers and 2D-grid them in (x, y) at 5 cm.
   Run connected components on the occupancy grid.
6. Reject components smaller than MIN_TABLE_AREA_M2 (default 0.10 m²).
7. Each surviving component is a Table.  Centre = xy mean, z = plane z,
   area = occupied cells * cell². Confidence scales with point count.

Walls are rejected because their normal is near-horizontal, so the z
histogram does not spike at a single value.  The floor is rejected by the
lower bound of the height band.  Small clutter (mugs, monitors) is rejected
by the area threshold.

MOCK
----
`MockLidarSight` synthesises a floor disc plus an optional rectangular table
at a configurable body-frame (x, y, z, width, length).  Same API surface.
Lets ACT-lite generate synthetic demos with varied table positions for
retrieval variety, and lets CI / off-robot tests run without hardware.
"""
from __future__ import annotations

import math
import os
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np


# ── URDF-documented LiDAR mount pose (torso_link ← mid360_link) ─────────────
#
# Source of truth: the bundled G1 URDF shared with `arm_fk`.  The G1 has no
# movable LiDAR — this is a fixed joint — so a single constant matrix is
# correct.  Read at import so updates to the URDF flow through automatically.
#
# Hard-coded fallbacks match the bundled URDF as of 2026-04-16 and are used
# only if the URDF file is missing (e.g. stripped-down deploy).
#
# PHYSICAL-MOUNT CORRECTION
# -------------------------
# The Livox MID-360 on this robot is physically mounted UPSIDE DOWN inside
# the crown / helmet assembly.  The URDF's mid360_joint says rpy="0 0.0401 0"
# — i.e. a LiDAR mounted upright with only a 2.3° nose-down pitch — but that
# is not what Unitree actually bolted to this chassis.  On 2026-04-17 we
# placed two ground-truth landmarks in front of the robot and the raw cloud
# lines up with reality only after a 180° roll about +X.
#
#   1. A bistro table whose top is 0.813 m above the floor appears in the
#      raw `rt/utlidar/cloud_livox_mid360` feed at z = +0.34 m — impossible
#      unless +z points DOWN, since the LiDAR sits roughly at torso level
#      and the table top is below it.  The floor itself appears at raw
#      z = +1.15 m (another dense spike), confirming LiDAR height ≈ 1.2 m.
#   2. A back-roller placed 0.5 m to the robot's LEFT (body +y) appears at
#      raw y = -0.48 m — impossible unless +y in the raw frame is actually
#      the robot's RIGHT.
#
# Both effects are explained by the physical LiDAR being rolled 180° about
# its +X axis relative to the URDF-specified orientation (probably a dome-
# down mount under the crown housing, or equivalently a +Z-down native
# convention in Unitree's `lidar_driver`).  We correct for this by ADDING π
# to the URDF-read roll, leaving the pitch and yaw alone.  The URDF file
# itself is left untouched so that arm_fk and anyone else reading it
# against a real-URDF-only tool keeps seeing the original document.
#
# If Unitree ever ships a driver update that outputs data in the URDF's
# declared frame (no roll-180° offset), set `_MOUNT_CORRECTION_ROLL_RAD` to
# 0.0.  This should be the ONLY place it needs to change.

_BUNDLED_URDF_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "arm_fk", "urdf", "g1_body29_hand14.urdf",
)
_FALLBACK_LIDAR_MOUNT_XYZ_M = (0.0002835, 0.00003, 0.41618)
_FALLBACK_LIDAR_MOUNT_RPY_RAD = (0.0, 0.04014257279586953, 0.0)

# Observed mount-orientation correction (see multi-paragraph comment above).
# π radians = 180° roll about +X, flipping both the raw Y and raw Z axes.
_MOUNT_CORRECTION_ROLL_RAD = math.pi


def _read_mid360_mount_from_urdf(
    path: str,
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    """Return (xyz, rpy) for the mid360_joint origin in the bundled URDF.

    Uses only `xml.etree.ElementTree` — no dependency on `arm_fk` (this
    module has to keep working if arm_fk is ever refactored).
    """
    tree = ET.parse(path)
    for j in tree.getroot().findall("joint"):
        if j.attrib.get("name") != "mid360_joint":
            continue
        origin = j.find("origin")
        if origin is None:
            break
        xyz = tuple(float(x) for x in origin.attrib.get("xyz", "0 0 0").split())
        rpy = tuple(float(x) for x in origin.attrib.get("rpy", "0 0 0").split())
        if len(xyz) != 3 or len(rpy) != 3:
            break
        return xyz, rpy
    raise ValueError("mid360_joint not found in URDF")


try:
    _URDF_LIDAR_MOUNT_XYZ_M, _URDF_LIDAR_MOUNT_RPY_RAD = _read_mid360_mount_from_urdf(
        _BUNDLED_URDF_PATH
    )
except Exception:  # noqa: BLE001 — URDF missing / malformed → use fallback
    _URDF_LIDAR_MOUNT_XYZ_M = _FALLBACK_LIDAR_MOUNT_XYZ_M
    _URDF_LIDAR_MOUNT_RPY_RAD = _FALLBACK_LIDAR_MOUNT_RPY_RAD

# What lidar_sight actually uses: URDF-read xyz, URDF-read pitch/yaw, but
# with π added to the roll to correct for the observed physical mount.
LIDAR_MOUNT_XYZ_M = _URDF_LIDAR_MOUNT_XYZ_M
LIDAR_MOUNT_RPY_RAD = (
    _URDF_LIDAR_MOUNT_RPY_RAD[0] + _MOUNT_CORRECTION_ROLL_RAD,
    _URDF_LIDAR_MOUNT_RPY_RAD[1],
    _URDF_LIDAR_MOUNT_RPY_RAD[2],
)

# For common-case use: URDF has roll=yaw=0 so the only non-trivial rotations
# are now (a) the correction roll and (b) the URDF pitch.
LIDAR_MOUNT_PITCH_RAD = LIDAR_MOUNT_RPY_RAD[1]

# ── Defaults for table detection ─────────────────────────────────────────────

VOXEL_SIZE_M = 0.05
MIN_TABLE_H_ABOVE_FLOOR_M = 0.55
MAX_TABLE_H_ABOVE_FLOOR_M = 1.15
TABLE_PLANE_BIN_M = 0.02
MIN_TABLE_PLANE_POINTS = 40
MIN_TABLE_AREA_M2 = 0.10
TABLE_CELL_M = 0.05

# ── Floor detection ──────────────────────────────────────────────────────────
#
# The LiDAR sees the floor as the lowest horizontal surface with a large
# support.  The exact torso-to-floor height depends on whether the robot is
# standing (~0.78 m), half-squat (~0.55 m) or seated (~0.30 m), so we find
# it from the data every frame.  If too few points are below torso to fit a
# floor (e.g. robot on a tabletop), `floor_z` returns None and table
# detection falls back to the absolute torso-frame band.

FLOOR_BIN_M = 0.05
FLOOR_MIN_SUPPORT = 200

# Absolute torso-frame fallback band used only when the floor cannot be
# detected.  A standing G1 torso_link is around 0.78 m above the floor, so a
# ~0.75 m table surface is about -0.03 m in torso z.  This fallback is
# intentionally wide.
TORSO_FALLBACK_BAND = (-0.35, 0.35)

# Absolute body-frame z band where tables actually land on THIS robot.
# Re-calibrated 2026-04-17 after discovering (and fixing) the mid360 frame's
# 180° roll offset described above.  With the corrected transform the
# standing-Regular-Mode G1 puts:
#
#   * the floor at body-z ≈ -0.78 m (torso_link sits ~0.78 m above floor)
#   * the 81.3 cm bistro table top at body-z ≈ +0.04 m
#   * a 0.30 m coffee table at body-z ≈ -0.48 m
#   * a 1.00 m kitchen counter at body-z ≈ +0.22 m
#   * a 1.10 m standing desk at body-z ≈ +0.32 m
#
# The (-0.55, +0.40) band covers coffee tables through standing desks with
# a safety margin below (floor at -0.78 is still 0.23 m outside the band)
# and above (walls / overhead structures filtered out).  Override with
# `find_tables(body_z_band=...)` when calibration changes or when running
# off-robot simulations where the floor is visible.
BODY_Z_TABLE_BAND = (-0.55, 0.40)


# ── Frame type ───────────────────────────────────────────────────────────────

@dataclass
class LidarCloud:
    """A single most-recent body-frame point cloud snapshot."""
    points: np.ndarray          # (N, 3) float32, body frame metres
    intensities: Optional[np.ndarray]  # (N,) float32, 0..1 or None
    timestamp: float            # time.monotonic() at frame arrival
    frame_id: str = "torso_link"


@dataclass
class Table:
    """A detected horizontal plane large enough to be a table."""
    center_xyz: Tuple[float, float, float]
    normal_xyz: Tuple[float, float, float]
    height_above_floor_m: Optional[float]
    area_estimate_m2: float
    confidence: float
    point_count: int


# ── Point-cloud parsing ──────────────────────────────────────────────────────

_POINT_DTYPE_LIVOX = np.dtype([
    ("x", "<f4"),
    ("y", "<f4"),
    ("z", "<f4"),
    ("intensity", "<f4"),
    ("ring", "<u2"),
    ("time", "<f4"),
])  # 22 bytes, matches the lidar_driver publisher we verified live


# Minimum radial range (in the LiDAR frame's xy plane) below which a point
# is treated as self-reflection.  The Unitree lidar_driver emits ~30k
# zero-range returns per Livox frame at r_xy=0 exactly — likely the face
# frame and chin assembly bouncing beams back into the dome (observed
# during bring-up).  Those pile up at the LiDAR origin in body frame and
# look like a fake dense plane to the flatness detector.
#
# The MID-360's published minimum range is 0.10 m; we set the threshold a
# touch higher to also catch any slightly-nonzero returns off the chin.
SELF_REFLECTION_MIN_RADIAL_M = 0.15


def _parse_pointcloud2(msg) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """Decode a sensor_msgs/PointCloud2 into (xyz float32 (N,3), intensity or None).

    Handles the 22-byte Livox layout fast-path via a dtype view.  Falls back
    to a general per-field decode if offsets differ so this keeps working if
    the firmware ever changes the struct.
    """
    buf = bytes(msg.data)
    step = int(msg.point_step)
    if step == 0 or len(buf) < step:
        return np.zeros((0, 3), dtype=np.float32), None
    npts = len(buf) // step

    # Fast-path: the field layout we've seen live on this robot.
    if step == 22 and len(msg.fields) >= 4 \
            and msg.fields[0].name == "x" and int(msg.fields[0].offset) == 0 \
            and msg.fields[3].name == "intensity":
        arr = np.frombuffer(buf[: npts * step], dtype=_POINT_DTYPE_LIVOX)
        xyz = np.stack([arr["x"], arr["y"], arr["z"]], axis=-1).astype(np.float32)
        intensity = arr["intensity"].astype(np.float32)
        return xyz, intensity

    # General fallback.  Only the common ROS PointField datatypes are
    # needed here (float32 for xyz, float32 or uint8 for intensity).
    raw = np.frombuffer(buf[: npts * step], dtype=np.uint8).reshape(npts, step)
    xyz_off = [0, 4, 8]  # sane default; overridden below
    intensity_off: Optional[int] = None
    for f in msg.fields:
        if f.name == "x": xyz_off[0] = int(f.offset)
        elif f.name == "y": xyz_off[1] = int(f.offset)
        elif f.name == "z": xyz_off[2] = int(f.offset)
        elif f.name == "intensity": intensity_off = int(f.offset)
    def _f32(off: int) -> np.ndarray:
        return np.frombuffer(raw[:, off:off + 4].tobytes(), dtype=np.float32)
    xyz = np.stack([_f32(xyz_off[0]), _f32(xyz_off[1]), _f32(xyz_off[2])], axis=-1)
    intensity = _f32(intensity_off) if intensity_off is not None else None
    return xyz.astype(np.float32), intensity


def _rpy_to_matrix(rpy: Tuple[float, float, float]) -> np.ndarray:
    """URDF RPY (roll about X, pitch about Y, yaw about Z), extrinsic X→Y→Z.

    Matches `arm_fk._rpy_to_matrix` so the two modules cannot disagree on
    frame conventions — if arm_fk ever changes its convention, so does this.
    """
    rx, ry, rz = rpy
    cr, sr = math.cos(rx), math.sin(rx)
    cp, sp = math.cos(ry), math.sin(ry)
    cy, sy = math.cos(rz), math.sin(rz)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr              ],
    ], dtype=np.float32)


_R_LIDAR_TO_BODY = _rpy_to_matrix(LIDAR_MOUNT_RPY_RAD)
_T_LIDAR_TO_BODY = np.asarray(LIDAR_MOUNT_XYZ_M, dtype=np.float32)


def _lidar_to_body(points_lidar: np.ndarray) -> np.ndarray:
    """Apply mid360_link → torso_link fixed transform to (N, 3) xyz."""
    return points_lidar @ _R_LIDAR_TO_BODY.T + _T_LIDAR_TO_BODY


# ── Connected-components labeller (pure numpy fallback for scipy) ────────────

def _connected_components_2d(binary: np.ndarray) -> Tuple[np.ndarray, int]:
    """Label 4-connected components.  Prefers scipy; pure-numpy fallback
    good enough for table-sized occupancy grids (up to a few hundred cells)."""
    try:
        from scipy.ndimage import label  # type: ignore
        return label(binary)
    except Exception:  # noqa: BLE001
        pass
    nx, ny = binary.shape
    labels = np.zeros((nx, ny), dtype=np.int32)
    parent = [0]
    def _find(a: int) -> int:
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    def _union(a: int, b: int):
        ra, rb = _find(a), _find(b)
        if ra == rb:
            return
        if ra < rb: parent[rb] = ra
        else:       parent[ra] = rb
    nxt = 1
    for i in range(nx):
        for j in range(ny):
            if not binary[i, j]:
                continue
            left  = labels[i, j - 1] if j > 0 else 0
            above = labels[i - 1, j] if i > 0 else 0
            if left == 0 and above == 0:
                labels[i, j] = nxt
                parent.append(nxt)
                nxt += 1
            elif left != 0 and above == 0:
                labels[i, j] = left
            elif above != 0 and left == 0:
                labels[i, j] = above
            else:
                mn = min(left, above)
                labels[i, j] = mn
                _union(left, above)
    remap: dict = {}
    for i in range(nx):
        for j in range(ny):
            if labels[i, j] == 0:
                continue
            root = _find(int(labels[i, j]))
            if root not in remap:
                remap[root] = len(remap) + 1
            labels[i, j] = remap[root]
    return labels, len(remap)


# ── Shared geometric primitives (used by real + mock paths) ──────────────────

def _voxel_downsample(points: np.ndarray, voxel_m: float) -> np.ndarray:
    """Return one representative point per ~voxel_m cube (grid centre)."""
    if points.shape[0] == 0:
        return points
    keys = np.round(points / voxel_m).astype(np.int64)
    # pack into one int64 key: assumes |coord/vox| < 2^20 which is fine for a
    # 5 cm voxel over ±50 km — plenty of headroom for a LiDAR scene.
    packed = (keys[:, 0] & 0xFFFFF) \
           | ((keys[:, 1] & 0xFFFFF) << 20) \
           | ((keys[:, 2] & 0xFFFFF) << 40)
    _, uniq = np.unique(packed, return_index=True)
    return points[uniq]


def _estimate_floor_z(points: np.ndarray) -> Optional[float]:
    """Lowest strongly-supported horizontal plane — or None.

    "Floor" here means "whatever horizontal surface is under the robot",
    which is usually the room floor but can be a raised platform if the
    robot is parked on one.  The heuristic: histogram all z values, walk
    from the lowest bin upward, and return the first bin with at least
    FLOOR_MIN_SUPPORT points.  Returns None only if no single bin has
    enough support — e.g. the robot is on the floor itself (LiDAR cannot
    look straight down), or the cloud is empty.
    """
    if points.shape[0] < FLOOR_MIN_SUPPORT:
        return None
    zs = points[:, 2]
    # Physically-plausible z window around the torso: ±2 m.  A 2 m
    # window covers standing (floor at z=-0.8), seated-on-table (floor at
    # z~+0.4), seated-on-desk (floor at z~+0.6), and the ceiling of a
    # typical indoor space without pulling in outlier noise.
    mask = (zs > -2.0) & (zs < 2.0)
    if mask.sum() < FLOOR_MIN_SUPPORT:
        return None
    zs = zs[mask]
    lo, hi = float(zs.min()), float(zs.max())
    nbins = max(4, int(math.ceil((hi - lo) / FLOOR_BIN_M)))
    counts, edges = np.histogram(zs, bins=nbins)
    for i in range(nbins):
        if counts[i] >= FLOOR_MIN_SUPPORT:
            return float(0.5 * (edges[i] + edges[i + 1]))
    return None


def _find_horizontal_planes(points: np.ndarray,
                             voxel_m: float,
                             z_lo: float,
                             z_hi: float,
                             min_plane_points: int,
                             min_area_m2: float,
                             cell_m: float,
                             max_plane_z_std_m: float,
                             slab_thickness_m: float = 0.10,
                             slab_step_m: float = 0.05) -> List[Table]:
    """Find every flat horizontal blob in the body-frame range [z_lo, z_hi].

    Slides a thin z-slab across the range (thickness `slab_thickness_m`,
    step `slab_step_m`) and runs 2D connected-components on each slab.
    Walls fail because in any single thin slab they occupy only a line of
    cells at one z — high z-std is not what rejects them, the POINT-COUNT
    AFTER THIN-SLABBING is.  Per-slab candidates are then de-duplicated
    across overlapping slabs (same plane ≈ same (x,y,z) within tolerance).
    """
    if points.shape[0] == 0:
        return []
    ds = _voxel_downsample(points, voxel_m)
    band_mask = (ds[:, 2] >= z_lo) & (ds[:, 2] <= z_hi)
    if band_mask.sum() < min_plane_points:
        return []
    band = ds[band_mask]

    x_lo_ = float(band[:, 0].min()) - cell_m
    x_hi_ = float(band[:, 0].max()) + cell_m
    y_lo_ = float(band[:, 1].min()) - cell_m
    y_hi_ = float(band[:, 1].max()) + cell_m
    nx = max(1, int(math.ceil((x_hi_ - x_lo_) / cell_m)))
    ny = max(1, int(math.ceil((y_hi_ - y_lo_) / cell_m)))
    if nx * ny > 200000:
        return []
    min_cells = max(1, int(math.ceil(min_area_m2 / (cell_m ** 2))))

    slab_centers = np.arange(float(z_lo) + slab_thickness_m / 2,
                             float(z_hi) - slab_thickness_m / 2 + slab_step_m,
                             slab_step_m)

    candidates: List[Table] = []
    for zc in slab_centers:
        s_lo = zc - slab_thickness_m / 2
        s_hi = zc + slab_thickness_m / 2
        slab_mask = (band[:, 2] >= s_lo) & (band[:, 2] <= s_hi)
        if slab_mask.sum() < min_plane_points:
            continue
        slab = band[slab_mask]

        xi = np.clip(np.floor((slab[:, 0] - x_lo_) / cell_m).astype(np.int32), 0, nx - 1)
        yi = np.clip(np.floor((slab[:, 1] - y_lo_) / cell_m).astype(np.int32), 0, ny - 1)
        occ = np.zeros((nx, ny), dtype=np.int32)
        np.add.at(occ, (xi, yi), 1)
        binary = occ > 0
        labels, nlbl = _connected_components_2d(binary)
        if nlbl == 0:
            continue
        point_labels = labels[xi, yi]

        for lbl in range(1, nlbl + 1):
            cells_in_blob = int((labels == lbl).sum())
            if cells_in_blob < min_cells:
                continue
            sel = point_labels == lbl
            n = int(sel.sum())
            if n < min_plane_points:
                continue
            blob = slab[sel]
            z_std = float(blob[:, 2].std())
            if z_std > max_plane_z_std_m:
                continue
            cx = float(blob[:, 0].mean())
            cy = float(blob[:, 1].mean())
            cz = float(np.median(blob[:, 2]))
            area = cells_in_blob * (cell_m ** 2)
            conf = max(0.0, min(1.0, n / 500.0))
            candidates.append(Table(
                center_xyz=(cx, cy, cz),
                normal_xyz=(0.0, 0.0, 1.0),
                height_above_floor_m=None,
                area_estimate_m2=float(area),
                confidence=float(conf),
                point_count=n,
            ))

    # Deduplicate overlapping slabs: cluster candidates by (cx, cy, cz) to
    # within `slab_thickness_m` in z and `cell_m * 2` in xy.  Keep the one
    # with the largest area per cluster.
    keep: List[Table] = []
    for c in sorted(candidates, key=lambda t: -t.area_estimate_m2):
        merged = False
        for k in keep:
            dz = abs(c.center_xyz[2] - k.center_xyz[2])
            dx = abs(c.center_xyz[0] - k.center_xyz[0])
            dy = abs(c.center_xyz[1] - k.center_xyz[1])
            if dz < slab_thickness_m and dx < cell_m * 6 and dy < cell_m * 6:
                merged = True
                break
        if not merged:
            keep.append(c)
    keep.sort(key=lambda t: (-t.confidence, -t.area_estimate_m2))
    return keep


def _find_tables_from_cloud(points: np.ndarray,
                            voxel_m: float,
                            min_h_above_floor_m: float,
                            max_h_above_floor_m: float,
                            table_plane_bin_m: float,
                            min_plane_points: int,
                            min_area_m2: float,
                            cell_m: float,
                            max_plane_z_std_m: float = 0.08) -> List[Table]:
    """Pure geometric table detector.  Shared between real + mock paths.

    The v1 algorithm: band-filter → xy-grid → connected components → reject
    blobs whose z-stddev is too high to be a flat surface.  `table_plane_bin_m`
    is accepted for API symmetry but no longer subdivides the band — doing
    so fragmented real tables whose z spread exceeded a single bin.
    """
    del table_plane_bin_m  # retained in signature for forward compat

    if points.shape[0] == 0:
        return []
    ds = _voxel_downsample(points, voxel_m)
    floor_z = _estimate_floor_z(ds)
    if floor_z is not None:
        band_lo = floor_z + min_h_above_floor_m
        band_hi = floor_z + max_h_above_floor_m
    else:
        band_lo, band_hi = TORSO_FALLBACK_BAND

    tables = _find_horizontal_planes(
        points, voxel_m, band_lo, band_hi, min_plane_points,
        min_area_m2, cell_m, max_plane_z_std_m,
    )
    if floor_z is not None:
        for i, t in enumerate(tables):
            tables[i] = Table(
                center_xyz=t.center_xyz,
                normal_xyz=t.normal_xyz,
                height_above_floor_m=t.center_xyz[2] - floor_z,
                area_estimate_m2=t.area_estimate_m2,
                confidence=t.confidence,
                point_count=t.point_count,
            )
    return tables


# ── Real (hardware-backed) LidarSight ────────────────────────────────────────

class LidarSight:
    """Singleton DDS subscriber for `rt/utlidar/cloud_livox_mid360`.

    Usage:
        lidar = LidarSight.instance()
        cloud = lidar.latest_cloud()
        for t in lidar.find_tables():
            print(t)
        lidar.shutdown()

    `instance()` is thread-safe and idempotent.  The first call spins up the
    capture thread and blocks until a frame arrives OR `warmup_timeout_s`
    expires.
    """

    _singleton_lock = threading.Lock()
    _singleton: "LidarSight | None" = None

    # --- Singleton access ---

    @classmethod
    def instance(cls,
                 topic: str = "rt/utlidar/cloud_livox_mid360",
                 warmup_timeout_s: float = 5.0,
                 accumulate_frames: int = 1) -> "LidarSight":
        with cls._singleton_lock:
            if cls._singleton is None:
                cls._singleton = cls.__new__(cls)
                cls._singleton._init(topic, warmup_timeout_s, accumulate_frames)
            return cls._singleton

    @classmethod
    def _reset_singleton_for_tests(cls):
        with cls._singleton_lock:
            if cls._singleton is not None:
                try:
                    cls._singleton.shutdown()
                except Exception:  # noqa: BLE001
                    pass
            cls._singleton = None

    # --- Init / lifecycle ---

    def _init(self, topic: str, warmup_timeout_s: float, accumulate_frames: int):
        # Import here so `from lidar_sight import MockLidarSight` works on
        # machines that do not have unitree_sdk2py installed (CI, dev laptop).
        try:
            from unitree_sdk2py.core.channel import (
                ChannelFactoryInitialize,
                ChannelSubscriber,
            )
            from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(
                "LidarSight: unitree_sdk2py is not importable.  Run under the "
                "`unitree_deploy` conda env on the robot, or use MockLidarSight "
                "for off-robot work."
            ) from exc

        self._topic = topic
        self._accumulate = max(1, int(accumulate_frames))

        self._lock = threading.Lock()
        self._points: Optional[np.ndarray] = None
        self._intensities: Optional[np.ndarray] = None
        self._timestamp: float = 0.0
        self._warm = threading.Event()

        # Frame-stitching across waist yaw.  Each accumulator slot
        # records the body-frame xyz already counter-rotated into the
        # leg/IMU frame using the *commanded* waist_yaw at frame
        # capture time.  The flash-scan blind-spot sweep flow
        # paused the subscriber during ramps and `resume`s with a
        # known commanded yaw at each held extreme — that's what
        # `_capture_waist_yaw` holds.  A live provider (set via
        # `set_waist_yaw_provider`) is the fallback for paths that
        # never pause: it reads `LowStateMonitor.waist_yaw` per frame.
        # When neither is set, no counter-rotation happens (waist
        # assumed centred).
        #
        # Sign: a point at (x_torso, y_torso) when the torso is at
        # waist_yaw=+θ relative to legs lives at leg-frame
        # (cos θ · x − sin θ · y,  sin θ · x + cos θ · y) — i.e. we
        # rotate BY +waist_yaw, not -waist_yaw.  (A pre-2026-05-13
        # build had the sign inverted, doubling the error.)
        self._waist_yaw_provider = None
        self._capture_waist_yaw = None  # set by resume(commanded_waist_yaw=…)
        # `paused` makes _on_msg drop incoming DDS frames.  Pause
        # during torso motion so smeared mid-ramp returns never enter
        # the accumulator.
        self._paused = False

        # Rolling accumulator for higher-density scans if requested.  Each
        # slot is an already-body-transformed xyz array.  Kept tight because
        # Livox frames are ~20k pts and memory is a real concern on the G1.
        self._acc_xyz: List[np.ndarray] = []
        self._acc_int: List[Optional[np.ndarray]] = []

        # ChannelFactoryInitialize is a process-global.  Peer modules
        # (e.g. depth_camera_sight) may already have called
        # it — duplicate calls raise, so guard with a module flag.
        global _CHANNEL_FACTORY_INITED
        if not _CHANNEL_FACTORY_INITED:
            try:
                ChannelFactoryInitialize(0)
                _CHANNEL_FACTORY_INITED = True
            except Exception:  # noqa: BLE001
                _CHANNEL_FACTORY_INITED = True

        self._running = True
        self._sub = ChannelSubscriber(topic, PointCloud2_)
        self._sub.Init(self._on_msg, 10)

        deadline = time.monotonic() + warmup_timeout_s
        while time.monotonic() < deadline:
            if self._warm.is_set():
                break
            time.sleep(0.02)
        if not self._warm.is_set():
            self.shutdown()
            raise RuntimeError(
                f"LidarSight: no message on `{topic}` within "
                f"{warmup_timeout_s:.1f}s.  Check: (1) `cyclonedds ps` shows "
                "a `lidar_driver` participant publishing that topic, and "
                "(2) $CYCLONEDDS_URI points at the same XML the driver uses."
            )

    # --- DDS callback ---

    def _on_msg(self, msg):
        # Flash-scan: drop frames while paused so torso-ramp
        # smear never enters the accumulator.
        if self._paused:
            return
        try:
            xyz_lidar, intensity = _parse_pointcloud2(msg)
        except Exception:  # noqa: BLE001
            return
        if xyz_lidar.shape[0] == 0:
            return
        # Self-reflection filter.  In the LiDAR's native frame, the robot's
        # face frame + chin assembly is physically very close to the dome,
        # so any return with near-zero radial distance is almost certainly
        # a beam that bounced off the robot's own head geometry.  Drop
        # these before doing the body-frame transform so downstream
        # consumers never see the fake dense blob at (0, 0, 0.416).
        r_xy_lidar = np.sqrt(xyz_lidar[:, 0] ** 2 + xyz_lidar[:, 1] ** 2)
        good = r_xy_lidar > SELF_REFLECTION_MIN_RADIAL_M
        if not good.all():
            xyz_lidar = xyz_lidar[good]
            if intensity is not None:
                intensity = intensity[good]
        if xyz_lidar.shape[0] == 0:
            return
        xyz_body = _lidar_to_body(xyz_lidar).astype(np.float32)

        # Counter-rotate torso-twisted frames into the leg/IMU
        # frame.  Prefer the commanded waist_yaw captured at resume()
        # time (exact, no servo overshoot, no AI-policy back-pressure
        # noise); fall back to the live provider for non-paused paths;
        # fall back to identity when neither is set.
        if self._capture_waist_yaw is not None:
            waist = float(self._capture_waist_yaw)
        elif self._waist_yaw_provider is not None:
            try:
                waist = float(self._waist_yaw_provider())
            except Exception:  # noqa: BLE001
                waist = 0.0
        else:
            waist = 0.0
        if abs(waist) > 1e-4:
            # Rotate BY +waist (NOT -waist — see _init comment): a
            # point at (x_torso, y_torso) when the torso is yawed +θ
            # relative to legs lives at leg-frame
            # (cos θ · x − sin θ · y,  sin θ · x + cos θ · y).
            c = math.cos(waist)
            s = math.sin(waist)
            x = xyz_body[:, 0]
            y = xyz_body[:, 1]
            xyz_body = np.stack(
                [x * c - y * s, x * s + y * c, xyz_body[:, 2]],
                axis=-1,
            ).astype(np.float32)

        ts = time.monotonic()
        if self._accumulate <= 1:
            with self._lock:
                self._points = xyz_body
                self._intensities = intensity
                self._timestamp = ts
        else:
            with self._lock:
                self._acc_xyz.append(xyz_body)
                self._acc_int.append(intensity)
                if len(self._acc_xyz) > self._accumulate:
                    self._acc_xyz.pop(0)
                    self._acc_int.pop(0)
                self._points = np.concatenate(self._acc_xyz, axis=0)
                if all(x is not None for x in self._acc_int):
                    self._intensities = np.concatenate(
                        [x for x in self._acc_int if x is not None], axis=0,
                    )
                else:
                    self._intensities = None
                self._timestamp = ts
        self._warm.set()

    # --- Public access ---

    def set_waist_yaw_provider(self, provider) -> None:
        """Fallback path: register a zero-arg callable returning the
        live waist_yaw (radians, positive = torso turned left) so this
        LidarSight instance can counter-rotate incoming frames into the
        leg/IMU frame *when not paused*.  Pass `LowStateMonitor.waist_yaw`
        (bound method) once the torque monitor is started.  Pass `None`
        to disable.

        Note: the flash-scan flow (pause + resume(commanded_waist_yaw))
        is preferred and overrides this provider while a captured value
        is in effect.  The provider is the fallback for non-pausing
        callers (e.g. continuous-monitoring uses where the torso may
        drift but never executes a deliberate sweep).
        """
        self._waist_yaw_provider = provider

    def pause(self) -> None:
        """Stop accumulating incoming LiDAR frames.

        DDS subscriber stays running; `_on_msg` just discards every
        frame until `resume()`.  Use this around any deliberate torso
        motion so mid-ramp frames (which are rotationally smeared
        within a single scan because a Livox frame spans ~100 ms while
        the waist may have moved several degrees) never enter the
        accumulator.
        """
        self._paused = True

    def resume(self, commanded_waist_yaw=None) -> None:
        """Re-enable frame accumulation.

        When `commanded_waist_yaw` is supplied, incoming frames are
        counter-rotated by that exact value (rotate BY +commanded_waist_yaw
        about +Z) — use this at each held extreme of a flash-scan
        sweep with the value passed to `set_waist_yaw(...)`.  When
        None, the per-frame counter-rotation falls back to the live
        provider (if any), then to identity.

        Pass `commanded_waist_yaw=0.0` (not None) when explicitly
        capturing at the upright pose — otherwise the live provider
        could read a small overshoot.
        """
        if commanded_waist_yaw is None:
            self._capture_waist_yaw = None
        else:
            self._capture_waist_yaw = float(commanded_waist_yaw)
        self._paused = False

    def flush_accumulator(self) -> None:
        """Discard buffered frames.  Useful around a flash-scan so
        stale rotationally-uncorrected frames from before the sweep
        don't contaminate the captured set.
        """
        with self._lock:
            self._acc_xyz = []
            self._acc_int = []

    def latest_cloud(self) -> Optional[LidarCloud]:
        """Non-blocking snapshot of the most-recent body-frame cloud."""
        with self._lock:
            pts = self._points
            it = self._intensities
            ts = self._timestamp
        if pts is None:
            return None
        return LidarCloud(points=pts, intensities=it, timestamp=ts)

    def shutdown(self):
        self._running = False
        sub = getattr(self, "_sub", None)
        if sub is not None:
            try:
                sub.Close()
            except Exception:  # noqa: BLE001
                pass

    # --- Geometric primitives ---

    def find_horizontal_planes(self,
                               voxel_m: float = VOXEL_SIZE_M,
                               z_lo: float = -2.0,
                               z_hi: float = 2.0,
                               min_plane_points: int = MIN_TABLE_PLANE_POINTS,
                               min_area_m2: float = MIN_TABLE_AREA_M2,
                               cell_m: float = TABLE_CELL_M,
                               max_plane_z_std_m: float = 0.08) -> List[Table]:
        """Return EVERY horizontal flat blob in the body-frame range [z_lo, z_hi].

        Unlike `find_tables()`, this does not estimate a floor and does not
        filter by "above-floor".  Use it to inspect all horizontal surfaces
        in the scene — the real floor, the deck the robot is parked on, any
        table, any shelf.  `height_above_floor_m` in the returned Tables is
        None because no floor is computed here.

        Intended for bring-up diagnostics and for callers that want to do
        their own posture-aware filtering.
        """
        cloud = self.latest_cloud()
        if cloud is None:
            return []
        return _find_horizontal_planes(
            cloud.points, voxel_m, z_lo, z_hi, min_plane_points,
            min_area_m2, cell_m, max_plane_z_std_m,
        )

    def find_tables(self,
                    voxel_m: float = VOXEL_SIZE_M,
                    min_h_above_floor_m: float = MIN_TABLE_H_ABOVE_FLOOR_M,
                    max_h_above_floor_m: float = MAX_TABLE_H_ABOVE_FLOOR_M,
                    table_plane_bin_m: float = TABLE_PLANE_BIN_M,
                    min_plane_points: int = MIN_TABLE_PLANE_POINTS,
                    min_area_m2: float = MIN_TABLE_AREA_M2,
                    cell_m: float = TABLE_CELL_M,
                    body_z_band: Optional[Tuple[float, float]] = BODY_Z_TABLE_BAND) -> List[Table]:
        """Return every table-shaped horizontal plane in the latest frame.

        By default uses an absolute body-frame z band tuned for this robot
        (see `BODY_Z_TABLE_BAND` docstring).  Pass `body_z_band=None` to
        fall back to the floor-relative band — useful on robots without
        heavy self-occlusion, or in simulation where the real floor is
        visible.  Thread-safe.
        """
        cloud = self.latest_cloud()
        if cloud is None:
            return []
        if body_z_band is not None:
            z_lo, z_hi = body_z_band
            return _find_horizontal_planes(
                cloud.points, voxel_m, z_lo, z_hi, min_plane_points,
                min_area_m2, cell_m, max_plane_z_std_m=0.08,
            )
        return _find_tables_from_cloud(
            cloud.points, voxel_m, min_h_above_floor_m, max_h_above_floor_m,
            table_plane_bin_m, min_plane_points, min_area_m2, cell_m,
        )

    def nearest_table_in_front(self,
                               max_distance_m: float = 5.0,
                               max_lateral_m: float = 1.5,
                               **kwargs) -> Optional[Table]:
        """Table with the smallest forward distance in the forward cone.

        "In front" means `center_xyz.x > 0` (forward) AND
        `abs(center_xyz.y) <= max_lateral_m` AND
        `hypot(x, y) <= max_distance_m`.
        """
        tables = self.find_tables(**kwargs)
        best: Optional[Table] = None
        best_d: float = float("inf")
        for t in tables:
            x, y, _ = t.center_xyz
            if x <= 0:
                continue
            if abs(y) > max_lateral_m:
                continue
            d = math.hypot(x, y)
            if d > max_distance_m:
                continue
            if d < best_d:
                best_d = d
                best = t
        return best

    def path_clear(self,
                   forward_m: float = 1.0,
                   half_width_m: float = 0.3,
                   z_band: Tuple[float, float] = (-0.6, 0.4)) -> Tuple[bool, Optional[Tuple[float, float, float]]]:
        """Is the forward corridor free of obstacles out to `forward_m`?

        Checks body-frame points with x in (0.05, forward_m], |y| <=
        half_width_m, and z in `z_band` (default "waist band" — ignores
        floor clutter and overhead lights).  Returns (clear, blocker_xyz).
        """
        cloud = self.latest_cloud()
        if cloud is None:
            return True, None  # "no data" → do not block callers
        p = cloud.points
        mask = (
            (p[:, 0] > 0.05) & (p[:, 0] <= forward_m)
            & (np.abs(p[:, 1]) <= half_width_m)
            & (p[:, 2] >= z_band[0]) & (p[:, 2] <= z_band[1])
        )
        if not mask.any():
            return True, None
        # Pick the closest blocker so the caller can steer around it.
        pts = p[mask]
        i = int(np.argmin(pts[:, 0]))
        return False, (float(pts[i, 0]), float(pts[i, 1]), float(pts[i, 2]))


_CHANNEL_FACTORY_INITED = False


# ── Mock implementation (no hardware, no DDS) ────────────────────────────────

class MockLidarSight:
    """Drop-in replacement for LidarSight without hardware or DDS.

    Synthesises a disc of floor points plus an optional rectangular table
    slab at a configurable body-frame position.  Matches the real sight's
    public surface exactly so test code can swap them via duck-typing.
    """

    def __init__(self,
                 floor_height_below_torso_m: float = 0.78,
                 floor_radius_m: float = 4.0,
                 n_floor: int = 4000,
                 table_xyz_m: Optional[Tuple[float, float, float]] = (1.0, 0.0, 0.04),
                 table_size_m: Tuple[float, float] = (0.52, 0.40),
                 n_table: int = 800,
                 noise_m: float = 0.01,
                 rng_seed: int = 0):
        # Defaults match the empirical G1 + bistro-table scene re-calibrated
        # on 2026-04-17 after correcting the mid360 frame 180° roll offset:
        # torso_link ≈ 0.78 m above floor in the standing Regular-Mode pose
        # (agreeing with depth_camera_sight's tape+SVD estimate within a
        # few cm), 81.3 cm table top at body-z ≈ +0.04 m, table top 52 × 40
        # cm.  Consumers needing a different scene pass overrides.
        self._floor_z = -float(floor_height_below_torso_m)
        self._floor_radius_m = float(floor_radius_m)
        self._n_floor = int(n_floor)
        self._table_xyz_m = tuple(table_xyz_m) if table_xyz_m is not None else None
        self._table_size_m = tuple(table_size_m)
        self._n_table = int(n_table)
        self._noise_m = float(noise_m)
        self._rng = np.random.default_rng(rng_seed)

    def _synth_points(self) -> Tuple[np.ndarray, np.ndarray]:
        rng = self._rng
        # Floor: points on a disc around the origin at z = floor_z.
        r = np.sqrt(rng.uniform(0.0, 1.0, self._n_floor)) * self._floor_radius_m
        th = rng.uniform(-math.pi, math.pi, self._n_floor)
        fx = r * np.cos(th)
        fy = r * np.sin(th)
        fz = np.full_like(fx, self._floor_z) + rng.normal(0.0, self._noise_m, self._n_floor)
        floor = np.stack([fx, fy, fz], axis=-1).astype(np.float32)
        intensity_floor = np.full(self._n_floor, 0.3, dtype=np.float32)

        if self._table_xyz_m is None:
            return floor, intensity_floor

        tx, ty, tz = self._table_xyz_m
        tw, tl = self._table_size_m
        jx = rng.uniform(-tw / 2, tw / 2, self._n_table)
        jy = rng.uniform(-tl / 2, tl / 2, self._n_table)
        jz = rng.normal(0.0, self._noise_m, self._n_table)
        table = np.stack([tx + jx, ty + jy, tz + jz], axis=-1).astype(np.float32)
        intensity_table = np.full(self._n_table, 0.6, dtype=np.float32)
        return (
            np.concatenate([floor, table], axis=0),
            np.concatenate([intensity_floor, intensity_table], axis=0),
        )

    def set_waist_yaw_provider(self, provider) -> None:
        """No-op on the mock — synthetic scenes are stitch-free by
        construction.  Defined for API parity with LidarSight.
        """
        return None

    def pause(self) -> None:
        return None

    def resume(self, commanded_waist_yaw=None) -> None:
        return None

    def flush_accumulator(self) -> None:
        return None

    def set_table(self, xyz_m: Optional[Tuple[float, float, float]],
                  size_m: Optional[Tuple[float, float]] = None):
        self._table_xyz_m = tuple(xyz_m) if xyz_m is not None else None
        if size_m is not None:
            self._table_size_m = tuple(size_m)

    def latest_cloud(self) -> LidarCloud:
        pts, it = self._synth_points()
        return LidarCloud(points=pts, intensities=it, timestamp=time.monotonic())

    def find_tables(self,
                    voxel_m: float = VOXEL_SIZE_M,
                    min_h_above_floor_m: float = MIN_TABLE_H_ABOVE_FLOOR_M,
                    max_h_above_floor_m: float = MAX_TABLE_H_ABOVE_FLOOR_M,
                    table_plane_bin_m: float = TABLE_PLANE_BIN_M,
                    min_plane_points: int = MIN_TABLE_PLANE_POINTS,
                    min_area_m2: float = MIN_TABLE_AREA_M2,
                    cell_m: float = TABLE_CELL_M,
                    body_z_band: Optional[Tuple[float, float]] = BODY_Z_TABLE_BAND) -> List[Table]:
        cloud = self.latest_cloud()
        if body_z_band is not None:
            z_lo, z_hi = body_z_band
            return _find_horizontal_planes(
                cloud.points, voxel_m, z_lo, z_hi, min_plane_points,
                min_area_m2, cell_m, max_plane_z_std_m=0.08,
            )
        return _find_tables_from_cloud(
            cloud.points, voxel_m, min_h_above_floor_m, max_h_above_floor_m,
            table_plane_bin_m, min_plane_points, min_area_m2, cell_m,
        )

    def find_horizontal_planes(self,
                               voxel_m: float = VOXEL_SIZE_M,
                               z_lo: float = -2.0,
                               z_hi: float = 2.0,
                               min_plane_points: int = MIN_TABLE_PLANE_POINTS,
                               min_area_m2: float = MIN_TABLE_AREA_M2,
                               cell_m: float = TABLE_CELL_M,
                               max_plane_z_std_m: float = 0.08) -> List[Table]:
        cloud = self.latest_cloud()
        return _find_horizontal_planes(
            cloud.points, voxel_m, z_lo, z_hi, min_plane_points,
            min_area_m2, cell_m, max_plane_z_std_m,
        )

    def nearest_table_in_front(self,
                               max_distance_m: float = 5.0,
                               max_lateral_m: float = 1.5,
                               **kwargs) -> Optional[Table]:
        return LidarSight.nearest_table_in_front(self, max_distance_m, max_lateral_m, **kwargs)

    def path_clear(self,
                   forward_m: float = 1.0,
                   half_width_m: float = 0.3,
                   z_band: Tuple[float, float] = (-0.6, 0.4)) -> Tuple[bool, Optional[Tuple[float, float, float]]]:
        return LidarSight.path_clear(self, forward_m, half_width_m, z_band)

    def shutdown(self):
        pass


# ── CLI diagnostic ───────────────────────────────────────────────────────────

def _diag_main():
    import argparse
    p = argparse.ArgumentParser(description="LiDAR Sight diagnostic")
    p.add_argument("--mock", action="store_true",
                   help="Use MockLidarSight (no hardware).")
    p.add_argument("--frames", type=int, default=10)
    p.add_argument("--accumulate", type=int, default=1,
                   help="Accumulate the last N frames for denser scans.")
    p.add_argument("--topic", default="rt/utlidar/cloud_livox_mid360")
    args = p.parse_args()

    if args.mock:
        lidar: object = MockLidarSight()
    else:
        lidar = LidarSight.instance(topic=args.topic,
                                     accumulate_frames=args.accumulate)

    for i in range(args.frames):
        c = lidar.latest_cloud()
        if c is None:
            print(f"[{i}] (no cloud yet)")
        else:
            p = c.points
            if p.shape[0] == 0:
                print(f"[{i}] empty")
            else:
                print(
                    f"[{i}] n={p.shape[0]:6d} "
                    f"x=[{p[:,0].min():+.2f},{p[:,0].max():+.2f}] "
                    f"y=[{p[:,1].min():+.2f},{p[:,1].max():+.2f}] "
                    f"z=[{p[:,2].min():+.2f},{p[:,2].max():+.2f}] "
                    f"ts={c.timestamp:.2f}"
                )
        time.sleep(0.1)

    tables = lidar.find_tables()
    print(f"find_tables: {len(tables)} table(s)")
    for j, t in enumerate(tables):
        print(
            f"  [{j}] center={t.center_xyz} "
            f"h_above_floor={t.height_above_floor_m} "
            f"area~{t.area_estimate_m2:.2f}m² "
            f"conf={t.confidence:.2f} n={t.point_count}"
        )

    nt = lidar.nearest_table_in_front()
    if nt is None:
        print("nearest_table_in_front: None")
    else:
        print(f"nearest_table_in_front: center={nt.center_xyz}")

    clear, blocker = lidar.path_clear(forward_m=1.5, half_width_m=0.3)
    print(f"path_clear(fwd=1.5): clear={clear} blocker={blocker}")

    lidar.shutdown()


if __name__ == "__main__":
    _diag_main()
