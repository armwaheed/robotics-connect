"""Build Isaac Sim sensor configs that share the REAL robot's envelope + blind spots.

Real-to-sim sensing: a simulator hands you idealized cameras and ray-casts, but the real robot's
sensors are tilted, range-limited, and **occluded by the robot's own body** in ways the sim won't
tell you. This module turns a robot descriptor's ``sensors`` (characterized on the hardware by the
``*_sight`` capability skills) into Isaac sensor configs that reproduce those exact characteristics,
so a detector trained in sim works on the robot.

Two builders, driven by the descriptor:

* :func:`camera_cfg_from_sensor` — a head ``CameraCfg`` at the real **down-tilt** (e.g. the G1 EDU's
  floor-plane-calibrated 51.29°), on the real ``mount_link``.
* :func:`lidar_cfg_from_sensor` + :func:`occlude` — a ``RayCaster`` reproducing the real LiDAR's FOV
  and the robot-body **blind spots** (face-frame azimuth bands, a chin elevation floor, dome
  self-reflection). RayCaster (mesh ray-cast) is the right fidelity: full RTX-Livox replay is neither
  affordable nor the point — the real device is already characterized on hardware; the sim only needs
  to share its blind spots.

Bed / large-flat-surface detection (:func:`detect_bed`) mirrors ``lidar_sight.find_tables`` widened
to a bed, and runs against the simulated cloud.

This is lifted from the proven implementation in armwaheed/robots#2
(``examples/isaac_bed_making/perception.py``, eye-verified on the G1) and generalized so the sensor
poses + occlusions come from the descriptor rather than being hard-coded. The G1 values remain the
defaults so the proven path is unchanged.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# ── Defaults (the proven G1 EDU values; overridden per-sensor by the descriptor) ──
DEFAULT_LIDAR_CHANNELS = 64
DEFAULT_LIDAR_HORIZONTAL_RES = 1.0                   # deg
DEFAULT_LIDAR_VFOV = (-45.0, 52.0)                   # deg
DEFAULT_LIDAR_HFOV = (-180.0, 180.0)                 # deg (360° dome)
DEFAULT_FACE_FRAME_AZIMUTH_DEG = (40.0, 45.0)        # ± bars blank these forward azimuth stripes
DEFAULT_CHIN_MAX_ELEVATION_DEG = -10.0               # chin blanks everything below this elevation
DEFAULT_SELF_REFLECTION_MIN_RADIAL_M = 0.15          # drop dome self-returns at the origin


@dataclass
class Occlusions:
    """The robot-body blind spots a simulated LiDAR must reproduce. Populated from the descriptor's
    ``sensors[].occlusions``; defaults are the G1 EDU's characterized values."""

    face_frame_azimuth_deg: tuple[float, float] | None = DEFAULT_FACE_FRAME_AZIMUTH_DEG
    chin_max_elevation_deg: float | None = DEFAULT_CHIN_MAX_ELEVATION_DEG
    self_reflection_min_radial_m: float | None = DEFAULT_SELF_REFLECTION_MIN_RADIAL_M

    @classmethod
    def from_descriptor_sensor(cls, sensor: dict) -> "Occlusions":
        """Read a descriptor ``sensors[]`` entry's ``occlusions`` list into the occlusion params."""
        face = chin = dome = None
        for occ in sensor.get("occlusions", []):
            kind = occ.get("kind")
            if kind == "azimuth_band" and occ.get("range_deg"):
                face = (float(occ["range_deg"][0]), float(occ["range_deg"][1]))
            elif kind == "elevation_floor" and occ.get("range_deg"):
                chin = float(occ["range_deg"][1])           # blank below the upper bound
            elif kind == "self_reflection" and occ.get("range_m"):
                dome = float(occ["range_m"][1])
        return cls(face, chin, dome)


@dataclass
class BedDetection:
    found: bool
    centroid_b: tuple[float, float, float] = (0.0, 0.0, 0.0)
    bearing_rad: float = 0.0
    range_m: float = 0.0
    area_m2: float = 0.0
    n_points: int = 0


# ── Bed / large-flat-surface detection (lidar_sight.find_tables, widened to a bed) ──
BED_Z_BAND_M = (-0.60, 0.40)        # LiDAR-frame height band that keeps furniture height, drops floor
BED_HORIZONTAL_TOL_M = 0.06         # |Δz| within a candidate plane to count as flat
BED_MIN_AREA_M2 = 1.2               # a bed's footprint dwarfs a table's (~0.15 m²)
BED_GRID_M = 0.10                   # occupancy-cell size for the area estimate


def _sensor_quat_from_tilt(tilt_deg: float) -> tuple[float, float, float, float]:
    """Quaternion (w, x, y, z) for a downward pitch ``tilt_deg`` about the body-Y axis."""
    half = math.radians(tilt_deg) / 2.0
    return (math.cos(half), 0.0, math.sin(half), 0.0)


def camera_cfg_from_sensor(prim_path: str, sensor: dict, width: int = 640, height: int = 480):
    """Isaac ``CameraCfg`` at the real head camera's mount + down-tilt, from a descriptor sensor entry.

    Uses ``sensor.pose.xyz_m`` and ``sensor.pose.tilt_deg`` (the floor-plane-calibrated downward pitch),
    so the simulated camera frames what the real one frames — the floor + near surface, not the room."""
    import isaaclab.sim as sim_utils
    from isaaclab.sensors import CameraCfg

    pose = sensor.get("pose", {})
    xyz = tuple(pose.get("xyz_m", (0.06, 0.0, 0.45)))
    tilt = float(pose.get("tilt_deg", 51.29))
    return CameraCfg(
        prim_path=prim_path,
        update_period=0,
        width=width,
        height=height,
        data_types=["rgb"],
        offset=CameraCfg.OffsetCfg(pos=xyz, rot=_sensor_quat_from_tilt(tilt), convention="world"),
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=18.0, focus_distance=400.0, horizontal_aperture=20.955,
            clipping_range=(0.05, 20.0),
        ),
    )


def lidar_cfg_from_sensor(prim_path: str, sensor: dict, mesh_prim_paths: list[str]):
    """Isaac ``RayCaster`` configured as the real LiDAR, from a descriptor sensor entry.

    Reads ``pose.xyz_m``, ``pose.tilt_deg`` (nose-down pitch), and ``fov`` from the descriptor. The
    self-occlusions in ``sensor.occlusions`` are applied at read time by :func:`occlude` — RayCaster
    casts geometric rays, so the body-blocked sectors are removed in post, matching the hardware."""
    from isaaclab.sensors import RayCasterCfg
    from isaaclab.sensors.ray_caster import patterns

    pose = sensor.get("pose", {})
    xyz = tuple(pose.get("xyz_m", (0.0003, 0.0, 0.416)))
    tilt = float(pose.get("tilt_deg", 2.3))
    fov = sensor.get("fov", {})
    vfov = tuple(fov.get("vertical_deg", DEFAULT_LIDAR_VFOV))
    hfov = tuple(fov.get("horizontal_deg", DEFAULT_LIDAR_HFOV))
    return RayCasterCfg(
        prim_path=prim_path,
        offset=RayCasterCfg.OffsetCfg(pos=xyz, rot=_sensor_quat_from_tilt(tilt)),
        ray_alignment="base",                                # rays follow the mount link's pose
        mesh_prim_paths=list(mesh_prim_paths),
        pattern_cfg=patterns.LidarPatternCfg(
            channels=DEFAULT_LIDAR_CHANNELS,
            vertical_fov_range=vfov,
            horizontal_fov_range=hfov,
            horizontal_res=DEFAULT_LIDAR_HORIZONTAL_RES,
        ),
        max_distance=12.0,
        debug_vis=False,
    )


def occlude(points_b: np.ndarray, occ: Occlusions | None = None) -> np.ndarray:
    """Drop the points the robot's own head geometry would block, given a cloud already in the LiDAR
    body frame (+x forward, +y left, +z up). Reproduces the descriptor's characterized blind spots
    (face-frame azimuth bands, chin elevation floor, dome self-reflection) so the simulated view
    matches the hardware. With ``occ=None`` the proven G1 EDU occlusions are used."""
    if points_b.shape[0] == 0:
        return points_b
    occ = occ or Occlusions()
    x, y, z = points_b[:, 0], points_b[:, 1], points_b[:, 2]
    radial = np.linalg.norm(points_b, axis=1)
    az = np.degrees(np.arctan2(y, x))                        # 0° = forward, ±180° = behind
    horiz = np.hypot(x, y)
    elev = np.degrees(np.arctan2(z, np.maximum(horiz, 1e-6)))
    drop = np.zeros(points_b.shape[0], dtype=bool)
    if occ.face_frame_azimuth_deg is not None:
        lo, hi = occ.face_frame_azimuth_deg
        drop |= (np.abs(az) >= lo) & (np.abs(az) <= hi)      # the ± vertical bars
    if occ.chin_max_elevation_deg is not None:
        drop |= elev < occ.chin_max_elevation_deg            # lower jaw blanks low elevations
    if occ.self_reflection_min_radial_m is not None:
        drop |= radial < occ.self_reflection_min_radial_m    # self-returns at the sensor origin
    return points_b[~drop]


def _to_body(points_w: np.ndarray, sensor_pos_w, sensor_quat_w) -> np.ndarray:
    """World hit points → LiDAR body frame (where the occlusions + detection work)."""
    w, x, y, z = sensor_quat_w
    r = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])
    return (points_w - np.asarray(sensor_pos_w)) @ r


def detect_bed(points_w: np.ndarray, sensor_pos_w, sensor_quat_w, occ: Occlusions | None = None) -> BedDetection:
    """Find a bed in a LiDAR frame. Mirrors lidar_sight.find_tables: occlude → height-band filter →
    keep the dominant horizontal plane → threshold on planar area. Returns the bed centre + bearing in
    the LiDAR body frame so a behaviour layer can steer the walk toward it. Eye-calibrate the z-band
    against the rendered cloud when wiring this for a new robot."""
    pts = _to_body(points_w, sensor_pos_w, sensor_quat_w)
    pts = pts[np.isfinite(pts).all(axis=1)]
    pts = occlude(pts, occ)
    band = pts[(pts[:, 2] >= BED_Z_BAND_M[0]) & (pts[:, 2] <= BED_Z_BAND_M[1])]
    if band.shape[0] < 20:
        return BedDetection(found=False)
    z_med = float(np.median(band[:, 2]))
    plane = band[np.abs(band[:, 2] - z_med) <= BED_HORIZONTAL_TOL_M]
    if plane.shape[0] < 20:
        return BedDetection(found=False)
    cells = np.unique(np.floor(plane[:, :2] / BED_GRID_M).astype(np.int64), axis=0)
    area = cells.shape[0] * BED_GRID_M * BED_GRID_M
    if area < BED_MIN_AREA_M2:
        return BedDetection(found=False)
    c = plane.mean(axis=0)
    return BedDetection(
        found=True,
        centroid_b=(float(c[0]), float(c[1]), float(c[2])),
        bearing_rad=float(math.atan2(c[1], c[0])),
        range_m=float(math.hypot(c[0], c[1])),
        area_m2=float(area),
        n_points=int(plane.shape[0]),
    )


def build_sensor_cfgs(descriptor: dict, robot_prim: str, mesh_prim_paths: list[str]) -> dict:
    """Build an Isaac sensor cfg for every sensor in a robot descriptor.

    Returns ``{sensor_name: cfg}`` — a ``CameraCfg`` for camera/depth_camera entries and a
    ``RayCaster`` for lidar entries, each mounted on the sensor's ``mount_link`` under ``robot_prim``
    and carrying the real pose/tilt/FOV. Occlusions are applied at read time via :func:`occlude` with
    :meth:`Occlusions.from_descriptor_sensor`. IMU / tactile / unknown types are skipped."""
    cfgs = {}
    for s in descriptor.get("sensors", []):
        name = s.get("name", s.get("type", "sensor"))
        link = s.get("mount_link", "torso_link")
        prim = f"{robot_prim}/{link}/{name}"
        if s.get("type") in ("camera", "depth_camera"):
            cfgs[name] = camera_cfg_from_sensor(prim, s)
        elif s.get("type") == "lidar":
            cfgs[name] = lidar_cfg_from_sensor(prim, s, mesh_prim_paths)
    return cfgs


def occlusions_for(descriptor: dict, sensor_name: str) -> Occlusions:
    """The :class:`Occlusions` for one named sensor in the descriptor (for use with :func:`occlude`)."""
    for s in descriptor.get("sensors", []):
        if s.get("name") == sensor_name:
            return Occlusions.from_descriptor_sensor(s)
    return Occlusions()
