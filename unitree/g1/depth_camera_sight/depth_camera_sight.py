#!/usr/bin/env python3
"""
Depth Camera Sight — singleton service for the G1 head Intel RealSense D435i.

Provides a single, import-and-call API so any
consumer (control modes, logging, dashboards) gets the same
frames from the same capture threads without fighting the device.

HARD CONSTRAINT: the G1 head camera is angled downward.  It sees only what
is directly in front of the robot — the floor, the near table surface, any
object on that table.  It does NOT see standing humans at conversational
range or the broader environment.  Consumers must be designed around that.

HYBRID CAPTURE (NOT NEGOTIABLE)
-------------------------------
On this robot the RealSense's color V4L2 node (/dev/video4) is permanently
held by `videohub_pc4`, the Unitree root service that feeds the phone app's
video preview.  We must NOT kill that service — it is load-bearing for the
operator's remote video monitoring.  Testing confirms:

    depth stream  (librealsense, /dev/video2-ish)      →  works
    color stream  (librealsense, /dev/video4)          →  Device or resource busy
    Unitree go2 VideoClient RGB over DDS               →  works  (1920x1080 MJPEG)

So this module runs TWO capture threads:

  1. librealsense depth-only pipeline (pyrealsense2, 640x480 z16 @ 30 Hz).
     Built from source against the system libusb / libudev — see README.
  2. unitree_sdk2py.go2.video.VideoClient RGB over DDS (1920x1080 MJPEG
     decoded via cv2.imdecode to BGR, converted to RGB to match DINOv2
     expectations).

The two streams are NOT pixel-aligned.  Depth geometry helpers
(pixel_to_body_xyz, pixel_to_table_xy) operate on DEPTH pixels using the
depth-sensor intrinsics.  RGB is exposed as-is for holistic features like
DINOv2 where pixel-precise alignment is irrelevant.  A future upgrade can
add optional alignment once a consumer actually needs it — see README.

CAPTURE SEMANTICS
-----------------
- Single-slot most-recent-frame buffer.  Consumers never block.
- Zero depth is invalid (Realsense convention).  Always mask out before use.
- Depth stored as float32 meters (raw uint16 * depth_scale applied once).
- Timestamps use time.monotonic() at frame arrival in this process.

MOCK
----
`MockDepthCameraSight` synthesises a plane at `table_height_m` with an
optional cylindrical object.  No hardware, no DDS, same API surface.
"""
from __future__ import annotations

import math
import os
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

try:
    import pyrealsense2 as rs  # noqa: F401
    _HAS_PYREALSENSE = True
except Exception:  # noqa: BLE001
    rs = None
    _HAS_PYREALSENSE = False

try:
    import cv2
    _HAS_CV2 = True
except Exception:  # noqa: BLE001
    cv2 = None
    _HAS_CV2 = False


# ── Camera geometry (see README for calibration procedure) ───────────────────

# Downward pitch of the head camera relative to body frame, in degrees.
# D435i optical frame convention: +X right, +Y down, +Z forward.  A
# positive tilt is a rotation that aims the optical axis into the floor.
#
# Calibrated on 2026-04-13 against the first G1 EDU unit via plane-SVD
# fit in `_calibrate_tilt.py`.  Mean of 5 consecutive runs: tilt 51.29°,
# camera-mount height 1.242 m above the floor, 0.83 cm RMS floor-plane
# residual.  Well inside the ±3 cm validation gate.
#
# KNOWN LIMITATION: the G1's balance mode makes continuous micro-pose
# adjustments; the head tilt drifts by ~1-2° on the timescale of minutes.
# A single calibrated constant is a simplification — the principled fix
# is to add the runtime neck pitch joint (from `rt/lowstate`) to this
# constant once the URDF gives us the right signs.  Until then,
# re-run `_calibrate_tilt.py` whenever the floor-plane fit residual in
# `_diag_camera_test.py` exceeds ~3 cm.
#
# This value is specific to one physical robot.  Re-calibrate when
# deploying to a new unit.
CAMERA_TILT_DEG_DEFAULT = 51.29  # calibrated 2026-04-13

# Defaults for the librealsense depth stream.  D435i depth supports 848x480
# and 640x480 natively; we default to 640x480 to minimise CPU cost in the
# ACT-lite hot path.  Consumers needing higher resolution can override at
# instance() time.
DEFAULT_WIDTH  = 640
DEFAULT_HEIGHT = 480
DEFAULT_FPS    = 30


# ── Frame type ───────────────────────────────────────────────────────────────

@dataclass
class SightFrame:
    """A single most-recent snapshot of what Depth Camera Sight has.

    `rgb` and `depth_m` may have DIFFERENT shapes — they come from different
    sensors (Unitree DDS for rgb, librealsense for depth) and the two
    streams are not spatially aligned at this layer.  Consumers that want
    pixel-precise alignment must do it themselves using the intrinsics
    dicts in `rgb_intrinsics` and `depth_intrinsics`.

    The geometry helpers (pixel_to_body_xyz, pixel_to_table_xy) operate on
    DEPTH pixels using `depth_intrinsics`.  This is the only sensible thing
    they can do without known depth→rgb extrinsics.
    """
    rgb: Optional[np.ndarray]        # (Hc, Wc, 3) uint8 RGB — or None if not yet warm
    depth_m: Optional[np.ndarray]    # (Hd, Wd) float32 meters.  0.0 = invalid.
    rgb_timestamp: float
    depth_timestamp: float
    rgb_intrinsics: Optional[dict]   # {"fx","fy","cx","cy","width","height"} or None
    depth_intrinsics: Optional[dict] # {"fx","fy","cx","cy","width","height","depth_scale"}
    tilt_deg: float


# ── Real (hardware-backed) DepthCameraSight ──────────────────────────────────

class DepthCameraSight:
    """Singleton service wrapping the D435i depth pipeline + Unitree RGB feed.

    Usage:
        cam = DepthCameraSight.instance()
        frame = cam.latest()
        if frame is not None and frame.depth_m is not None:
            ...
        cam.shutdown()

    The first call to `instance()` starts both capture threads and blocks
    until either stream has warmed up OR `warmup_timeout_s` elapses.  If
    only one stream warms up in time the frame is still returned with the
    missing side as None — the depth-only degraded mode is useful even when
    the Unitree DDS is temporarily down.
    """

    _singleton_lock = threading.Lock()
    _singleton: "DepthCameraSight | None" = None

    # --- Singleton access ---

    @classmethod
    def instance(cls,
                 width: int = DEFAULT_WIDTH,
                 height: int = DEFAULT_HEIGHT,
                 fps: int = DEFAULT_FPS,
                 tilt_deg: float = CAMERA_TILT_DEG_DEFAULT,
                 warmup_timeout_s: float = 5.0,
                 enable_rgb: bool = True,
                 enable_depth: bool = True) -> "DepthCameraSight":
        with cls._singleton_lock:
            if cls._singleton is None:
                cls._singleton = cls.__new__(cls)
                cls._singleton._init(width, height, fps, tilt_deg,
                                     warmup_timeout_s, enable_rgb, enable_depth)
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

    def _init(self, width: int, height: int, fps: int,
              tilt_deg: float, warmup_timeout_s: float,
              enable_rgb: bool, enable_depth: bool):
        if enable_depth and not _HAS_PYREALSENSE:
            raise RuntimeError(
                "DepthCameraSight: pyrealsense2 is not importable.  Build "
                "it from source against the system librealsense (see the "
                "README for the procedure) or call "
                "DepthCameraSight.instance(enable_depth=False)."
            )
        if enable_rgb and not _HAS_CV2:
            raise RuntimeError(
                "DepthCameraSight: cv2 is required for RGB JPEG decoding "
                "from Unitree DDS.  Install opencv-python or call "
                "DepthCameraSight.instance(enable_rgb=False)."
            )

        self._tilt_deg = tilt_deg
        self._enable_rgb = enable_rgb
        self._enable_depth = enable_depth
        self._running = True

        # Separate locks so an RGB and a depth capture never contend.
        self._rgb_lock = threading.Lock()
        self._depth_lock = threading.Lock()

        self._rgb: Optional[np.ndarray] = None
        self._rgb_ts: float = 0.0
        self._rgb_intr: Optional[dict] = None

        self._depth_m: Optional[np.ndarray] = None
        self._depth_ts: float = 0.0
        self._depth_intr: Optional[dict] = None

        self._rgb_warm = threading.Event()
        self._depth_warm = threading.Event()

        self._threads: list[threading.Thread] = []

        if enable_depth:
            self._start_depth_pipeline(width, height, fps)
            t = threading.Thread(target=self._depth_capture_loop,
                                 name="DepthCameraSight-depth",
                                 daemon=True)
            t.start()
            self._threads.append(t)

        if enable_rgb:
            self._start_rgb_client()
            t = threading.Thread(target=self._rgb_capture_loop,
                                 name="DepthCameraSight-rgb",
                                 daemon=True)
            t.start()
            self._threads.append(t)

        # Block until at least one stream has warmed up OR we time out.
        deadline = time.monotonic() + warmup_timeout_s
        while time.monotonic() < deadline:
            if (enable_depth and self._depth_warm.is_set()) or \
               (enable_rgb   and self._rgb_warm.is_set()):
                break
            time.sleep(0.02)
        if not (self._depth_warm.is_set() or self._rgb_warm.is_set()):
            self.shutdown()
            raise RuntimeError(
                f"DepthCameraSight: no frame within {warmup_timeout_s:.1f}s. "
                "Check `fuser /dev/video*` and whether videohub_pc4 is "
                "running (expected: yes, and it must not block the depth "
                "video node)."
            )

    # --- librealsense depth path ---

    def _start_depth_pipeline(self, width: int, height: int, fps: int):
        self._rs_pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        profile = self._rs_pipeline.start(cfg)
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = float(depth_sensor.get_depth_scale())
        depth_stream = profile.get_stream(rs.stream.depth).as_video_stream_profile()
        di = depth_stream.get_intrinsics()
        self._depth_intr = {
            "fx": float(di.fx), "fy": float(di.fy),
            "cx": float(di.ppx), "cy": float(di.ppy),
            "width": int(di.width), "height": int(di.height),
            "depth_scale": depth_scale,
        }

    def _depth_capture_loop(self):
        depth_scale = self._depth_intr["depth_scale"]
        while self._running:
            try:
                frames = self._rs_pipeline.wait_for_frames(timeout_ms=1000)
            except Exception:  # noqa: BLE001
                continue
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                continue
            raw = np.asanyarray(depth_frame.get_data())  # uint16 (H,W)
            dm = raw.astype(np.float32) * depth_scale
            ts = time.monotonic()
            with self._depth_lock:
                self._depth_m = dm
                self._depth_ts = ts
            self._depth_warm.set()

    # --- Unitree DDS RGB path ---

    def _start_rgb_client(self):
        # unitree_sdk2py lives as an editable pip install under
        # /home/unitree/unitree_sdk2_python — it's already on the path in the
        # `unitree_deploy` env that runs the modes.  Importing here (not at
        # module top-level) keeps the mock path importable off-robot.
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.go2.video.video_client import VideoClient

        # ChannelFactoryInitialize is a global.  If a caller already
        # initialised it (several control modes do), a
        # second call raises or silently breaks.  Guard it with a module
        # flag.
        global _CHANNEL_FACTORY_INITED
        if not _CHANNEL_FACTORY_INITED:
            try:
                ChannelFactoryInitialize(0)
                _CHANNEL_FACTORY_INITED = True
            except Exception:  # noqa: BLE001
                # Assume already initialised by another component.
                _CHANNEL_FACTORY_INITED = True

        self._video_client = VideoClient()
        self._video_client.SetTimeout(3.0)
        self._video_client.Init()

    def _rgb_capture_loop(self):
        # VideoClient.GetImageSample returns (code, bytes).  It's
        # server-pushed so the call rate is bounded by the Unitree hub's
        # frame rate (~15 Hz nominal); we sleep briefly on misses to avoid
        # hot-spinning if the DDS stops publishing.
        while self._running:
            try:
                code, data = self._video_client.GetImageSample()
            except Exception:  # noqa: BLE001
                time.sleep(0.1)
                continue
            if code != 0 or not data:
                time.sleep(0.05)
                continue
            arr = np.frombuffer(bytes(data), dtype=np.uint8)
            bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if bgr is None:
                continue
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            h, w = rgb.shape[:2]
            if self._rgb_intr is None:
                # Unitree VideoClient does not expose a calibrated intrinsic
                # matrix.  Use a placeholder based on the frame size so
                # geometry helpers that accidentally get handed rgb pixels
                # at least return sane values.  Real object localisation
                # should go through depth pixels.
                self._rgb_intr = {
                    "fx": float(w), "fy": float(w),
                    "cx": w / 2.0, "cy": h / 2.0,
                    "width": w, "height": h,
                }
            ts = time.monotonic()
            with self._rgb_lock:
                self._rgb = rgb
                self._rgb_ts = ts
            self._rgb_warm.set()

    # --- Public access ---

    def latest(self) -> SightFrame:
        """Return a snapshot of the most recent rgb + depth frames.

        Fields are None if that stream hasn't warmed up yet; consumers
        should always check.  This never blocks.
        """
        with self._rgb_lock:
            rgb, rgb_ts, rgb_intr = self._rgb, self._rgb_ts, self._rgb_intr
        with self._depth_lock:
            dm, dm_ts, dm_intr = self._depth_m, self._depth_ts, self._depth_intr
        return SightFrame(
            rgb=rgb,
            depth_m=dm,
            rgb_timestamp=rgb_ts,
            depth_timestamp=dm_ts,
            rgb_intrinsics=rgb_intr,
            depth_intrinsics=dm_intr,
            tilt_deg=self._tilt_deg,
        )

    def shutdown(self):
        """Stop capture and release the device."""
        self._running = False
        for t in self._threads:
            try:
                t.join(timeout=2.0)
            except Exception:  # noqa: BLE001
                pass
        try:
            if hasattr(self, "_rs_pipeline"):
                self._rs_pipeline.stop()
        except Exception:  # noqa: BLE001
            pass

    # --- Geometry helpers (operate on DEPTH pixels) ---

    def pixel_to_camera_xyz(self, u: int, v: int,
                             depth_m_override: Optional[float] = None) -> Optional[np.ndarray]:
        """Back-project a DEPTH pixel (u, v) into the camera optical frame.

        Camera optical frame: +X right, +Y down, +Z forward.
        Returns None if the depth at (u, v) is invalid.
        """
        frame = self.latest()
        if frame.depth_m is None or frame.depth_intrinsics is None:
            return None
        if depth_m_override is not None:
            d = float(depth_m_override)
        else:
            d = float(frame.depth_m[v, u])
        if d <= 0.0:
            return None
        fx = frame.depth_intrinsics["fx"]
        fy = frame.depth_intrinsics["fy"]
        cx = frame.depth_intrinsics["cx"]
        cy = frame.depth_intrinsics["cy"]
        return np.array([(u - cx) * d / fx,
                          (v - cy) * d / fy,
                          d], dtype=np.float32)

    def pixel_to_body_xyz(self, u: int, v: int,
                          depth_m_override: Optional[float] = None) -> Optional[np.ndarray]:
        """Back-project a DEPTH pixel (u, v) into body frame.

        Body frame: +X forward, +Y left, +Z up.  Origin at the camera mount.

        Derivation.  At 0° tilt the camera optical frame maps to body as:
            cam_X (right) →  -body_Y
            cam_Y (down)  →  -body_Z
            cam_Z (fwd)   →  +body_X
        A downward pitch of `t` rotates the camera about body +Y by +t
        (nose down), so cam_Z moves from +body_X to (+body_X, -body_Z) and
        cam_Y moves from -body_Z to (-body_X, -body_Z).  Composing:
            body_x = -cy·sin(t) + cz·cos(t)
            body_y = -cx
            body_z = -cy·cos(t) - cz·sin(t)
        Sanity check: at t=30°, (cx=0, cy=0, cz=d) → body=(0.866·d, 0, -0.5·d).
        The camera is looking forward-and-down, so body_z is negative —
        what it sees is below the camera mount.
        """
        cam = self.pixel_to_camera_xyz(u, v, depth_m_override)
        if cam is None:
            return None
        cx_, cy_, cz_ = float(cam[0]), float(cam[1]), float(cam[2])
        t = math.radians(self._tilt_deg)
        ct, st = math.cos(t), math.sin(t)
        body_x = -cy_ * st + cz_ * ct
        body_y = -cx_
        body_z = -cy_ * ct - cz_ * st
        return np.array([body_x, body_y, body_z], dtype=np.float32)

    # Physical range clip for the body-Z grid.  The head camera is
    # ~1.24 m above the floor in the calibrated stance and is
    # pointed down-and-forward, so any body-Z outside this range is
    # either a saturated-depth outlier (D435i reports ~65.535 m for
    # "no reading" and those back-project to body-Z ~-70 m after the
    # tilt rotation) or a ray that has left the physical scene (walls
    # farther than 10 m, anything above the shoulders).  Clipping
    # here keeps the histogram in `table_plane_z` honest and keeps the
    # old median codepath's outlier-robustness.
    _BODY_Z_MIN_M = -2.0
    _BODY_Z_MAX_M =  0.5
    _DEPTH_MAX_M  = 10.0

    def _sample_body_z_grid(self,
                             sample_stride: int = 8,
                             valid_frac_min: float = 0.2) -> Optional[np.ndarray]:
        """Back-project a coarse depth-pixel grid into body-frame Z.

        Internal helper shared by `table_plane_z()` and the
        `table_plane_z_diag()` debug utility.  Returns the 1-D array of
        valid body-frame Z values (saturated depth and
        out-of-physical-range body-Z filtered out), or None if fewer
        than `valid_frac_min` of the grid pixels survive the filter.
        """
        frame = self.latest()
        if frame.depth_m is None or frame.depth_intrinsics is None:
            return None
        depth = frame.depth_m
        h, w = depth.shape
        fx = frame.depth_intrinsics["fx"]
        fy = frame.depth_intrinsics["fy"]
        cx0 = frame.depth_intrinsics["cx"]
        cy0 = frame.depth_intrinsics["cy"]
        us = np.arange(0, w, sample_stride, dtype=np.float32)
        vs = np.arange(h // 5, h, sample_stride, dtype=np.float32)
        uu, vv = np.meshgrid(us, vs)
        dd = depth[vv.astype(np.int32), uu.astype(np.int32)]
        # Valid depth: strictly positive AND within the physical range.
        # The second clause filters the D435i's "no reading" sentinel
        # (~65.535 m) which used to slip through a `dd > 0` test and
        # pollute the histogram with body_z ≈ -70 m outliers.
        valid = (dd > 0.0) & (dd < self._DEPTH_MAX_M)
        if valid.sum() < max(16, int(valid_frac_min * dd.size)):
            return None
        uu = uu[valid]; vv = vv[valid]; dd = dd[valid]
        cam_y = (vv - cy0) * dd / fy
        cam_z = dd
        t = math.radians(self._tilt_deg)
        ct, st = math.cos(t), math.sin(t)
        body_z = -cam_y * ct - cam_z * st
        # Physical-range clip on the body-frame Z.  Anything outside
        # this window is either a saturated outlier that slipped
        # through, a reflection, or a ceiling pixel — none of which
        # are a table surface in front of the robot.
        in_range = (body_z >= self._BODY_Z_MIN_M) & (body_z <= self._BODY_Z_MAX_M)
        if in_range.sum() < max(16, int(valid_frac_min * dd.size * 0.5)):
            return None
        return body_z[in_range].astype(np.float32)

    def table_plane_z(self,
                      sample_stride: int = 8,
                      valid_frac_min: float = 0.2,
                      bin_size_m: float = 0.02,
                      min_support_frac: float = 0.03) -> Optional[float]:
        """Estimate the NEAREST horizontal surface Z in body frame.

        Back-projects a coarse grid of depth pixels into body frame,
        histograms the body-Z values, and returns the centre of the
        highest-Z (closest to camera) bin that has at least
        `min_support_frac` of the grid's samples supporting it.

        This is the fix for the live-camera bring-up observation
        that a median-body-Z formulation returned the FLOOR rather than
        a small table in front of the robot: with a 45%-coverage
        paint-palette table at 0.41 m below camera and the remaining
        55% of the frame being floor at 1.22 m below camera plus a
        yoga mat / pillow at similar depth, the median landed at
        ~-1.09 m (dominated by the floor) instead of -0.41 m (the
        actual table).  Histogramming and picking the highest-support
        top bin fixes that without needing to know the floor height or
        the table geometry a priori.

        Parameters
        ----------
        sample_stride : pixel stride of the back-projection grid.
        valid_frac_min : minimum fraction of the grid that must have
            valid depth before we return anything (else None).
        bin_size_m : histogram bin width in metres.  2 cm is fine for
            typical table surfaces; drop to 1 cm if you need to resolve
            very shallow objects.
        min_support_frac : a bin must contain at least this fraction of
            the grid's valid samples to be considered a "real" surface.
            5 % catches small tables (like a paint palette at 25 % of
            the grid, after floor-dominated bins are skipped) without
            firing on depth noise.

        Returns
        -------
        The body-frame Z (metres, negative for surfaces below camera
        mount) of the nearest-to-camera surface with enough support, or
        None if the depth frame is too sparse / the grid has no bin
        with enough support.
        """
        body_z = self._sample_body_z_grid(sample_stride, valid_frac_min)
        if body_z is None or body_z.size < 16:
            return None
        min_z = float(body_z.min())
        max_z = float(body_z.max())
        if max_z - min_z < 1e-6:
            return float(max_z)
        n_bins = max(3, int(math.ceil((max_z - min_z) / bin_size_m)))
        counts, edges = np.histogram(body_z, bins=n_bins)
        # Support threshold: at least `min_support_frac` of the grid
        # samples, but never fewer than 30 absolute (protects against
        # single-pixel noise spikes regardless of grid size — a 30-
        # sample bin corresponds to a ~15 x 15 cm patch at typical
        # table distance, which is the smallest "real surface" we
        # ever care about).
        min_support = max(30, int(min_support_frac * body_z.size))
        # Scan from the highest (nearest-camera) bin downward.  Return
        # the centre of the first bin with enough support.
        for i in range(n_bins - 1, -1, -1):
            if counts[i] >= min_support:
                return float(0.5 * (edges[i] + edges[i + 1]))
        return None

    def table_plane_z_diag(self,
                            sample_stride: int = 8,
                            valid_frac_min: float = 0.2,
                            bin_size_m: float = 0.02) -> Optional[dict]:
        """Debug view of `table_plane_z`.

        Returns a dict with the full histogram, percentiles, and the
        picked table Z so operators can sanity-check the surface
        selection against what the camera is physically seeing.  Used
        by `arm_fk/smoke_live_table.py`.  Never called in the
        control hot path.
        """
        body_z = self._sample_body_z_grid(sample_stride, valid_frac_min)
        if body_z is None:
            return None
        min_z = float(body_z.min())
        max_z = float(body_z.max())
        n_bins = max(3, int(math.ceil(max(max_z - min_z, bin_size_m) / bin_size_m)))
        counts, edges = np.histogram(body_z, bins=n_bins)
        return {
            "n_samples": int(body_z.size),
            "min_z": min_z,
            "max_z": max_z,
            "median_z": float(np.median(body_z)),
            "p25": float(np.percentile(body_z, 25)),
            "p50": float(np.percentile(body_z, 50)),
            "p75": float(np.percentile(body_z, 75)),
            "p90": float(np.percentile(body_z, 90)),
            "p95": float(np.percentile(body_z, 95)),
            "p99": float(np.percentile(body_z, 99)),
            "bin_edges": edges.tolist(),
            "bin_counts": counts.tolist(),
            "table_plane_z": self.table_plane_z(
                sample_stride=sample_stride,
                valid_frac_min=valid_frac_min,
                bin_size_m=bin_size_m,
            ),
        }

    def table_plane_xyz(self,
                         sample_stride: int = 8,
                         valid_frac_min: float = 0.2,
                         bin_size_m: float = 0.02,
                         min_support_frac: float = 0.03,
                         z_tolerance_m: float = 0.04) -> Optional[tuple]:
        """Like `table_plane_z()` but also returns the XY centroid of the
        detected surface in body frame.

        The centroid is computed over the depth-grid pixels whose
        back-projected body_z is within `z_tolerance_m` of the picked
        plane Z.  It lets a caller test "is the detected table roughly
        centred in the forward cone of the camera?" before committing
        to a slow arm descent / blind sweep.

        Returns `(plane_z, centroid_x, centroid_y)` in body-frame
        metres (+X forward, +Y left, origin at camera mount), or None
        when no surface is found (same criteria as `table_plane_z`).
        A caller can use it in a goal-approach loop to decide whether to
        step closer or commit to the next phase.
        """
        frame = self.latest()
        if frame.depth_m is None or frame.depth_intrinsics is None:
            return None
        depth = frame.depth_m
        h, w = depth.shape
        fx = float(frame.depth_intrinsics["fx"])
        fy = float(frame.depth_intrinsics["fy"])
        cx0 = float(frame.depth_intrinsics["cx"])
        cy0 = float(frame.depth_intrinsics["cy"])

        us = np.arange(0, w, sample_stride, dtype=np.float32)
        vs = np.arange(h // 5, h, sample_stride, dtype=np.float32)
        uu, vv = np.meshgrid(us, vs)
        dd = depth[vv.astype(np.int32), uu.astype(np.int32)]
        valid = (dd > 0.0) & (dd < self._DEPTH_MAX_M)
        if valid.sum() < max(16, int(valid_frac_min * dd.size)):
            return None
        uu_v = uu[valid]
        vv_v = vv[valid]
        dd_v = dd[valid]

        cam_x = (uu_v - cx0) * dd_v / fx
        cam_y = (vv_v - cy0) * dd_v / fy
        cam_z = dd_v
        t = math.radians(self._tilt_deg)
        ct, st = math.cos(t), math.sin(t)
        body_x = -cam_y * st + cam_z * ct
        body_y = -cam_x
        body_z = -cam_y * ct - cam_z * st

        in_range = (body_z >= self._BODY_Z_MIN_M) & (body_z <= self._BODY_Z_MAX_M)
        if in_range.sum() < max(16, int(valid_frac_min * dd.size * 0.5)):
            return None
        body_x = body_x[in_range]
        body_y = body_y[in_range]
        body_z = body_z[in_range]

        min_z = float(body_z.min())
        max_z = float(body_z.max())
        if max_z - min_z < 1e-6:
            plane_z = float(max_z)
        else:
            n_bins = max(3, int(math.ceil((max_z - min_z) / bin_size_m)))
            counts, edges = np.histogram(body_z, bins=n_bins)
            min_support = max(30, int(min_support_frac * body_z.size))
            plane_z = None
            for i in range(n_bins - 1, -1, -1):
                if counts[i] >= min_support:
                    plane_z = float(0.5 * (edges[i] + edges[i + 1]))
                    break
            if plane_z is None:
                return None

        # Centroid over pixels whose body_z is within z_tolerance_m of
        # the picked plane — these are "surface" pixels.
        surface = np.abs(body_z - plane_z) <= z_tolerance_m
        if surface.sum() < 16:
            return None
        centroid_x = float(body_x[surface].mean())
        centroid_y = float(body_y[surface].mean())
        return (plane_z, centroid_x, centroid_y)

    # ── Object localisation on the table plane ─────────────────────────

    # Body-frame envelope for object detection.  Set wide so ANY object
    # the depth camera can see above the table plane is a candidate; the
    # depth-above-table filter + connected-component sizing + center-biased
    # tiebreak are the real discrimination, not the crop.
    #
    # Removed the tight (0.20, 0.55) × (±0.30) crop after live
    # calibration showed bottles at body_x ≈ 0.19 slipping under the
    # lower bound.  The physical-range clip on body_z (±2 m) already
    # excludes depth noise that back-projects outside the robot's vicinity.
    _OBJECT_X_CROP_M = (-1.0, 3.0)
    _OBJECT_Y_CROP_M = (-3.0, 3.0)
    _OBJECT_MIN_HEIGHT_ABOVE_TABLE_M = 0.05
    _OBJECT_GRID_CELL_M = 0.02
    _OBJECT_MIN_CELL_POINTS = 2   # cells with fewer points are noise
    _OBJECT_MIN_CELLS = 3         # ignore blobs smaller than this many cells
    _OBJECT_SAMPLE_STRIDE = 4     # depth-pixel stride on the back-projection

    def object_xyz_on_table_m(
        self,
        min_height_above_table_m: float = _OBJECT_MIN_HEIGHT_ABOVE_TABLE_M,
        x_crop_m: tuple = _OBJECT_X_CROP_M,
        y_crop_m: tuple = _OBJECT_Y_CROP_M,
        sample_stride: int = _OBJECT_SAMPLE_STRIDE,
    ) -> Optional[tuple]:
        """Return the (x, y, z) body-frame centroid of the largest object
        sitting on the table, or None if no spike is found.

        Detects objects by back-projecting a
        coarse grid of depth pixels into body frame, filtering to the
        reach envelope, dropping everything at or below
        `table_plane_z() + min_height_above_table_m`, 2D-gridding the
        survivors, running connected-components on the occupancy grid, and
        returning the centroid of the best blob.

        Blob selection:
          - zero blobs  → None (caller falls back to blind sweep)
          - one blob    → return it
          - 2+ blobs    → pick the one whose XY centroid is closest to
                          (midpoint_of_x_crop, 0); tiebreak by smaller x.

        No ML, no HSV, no colour.  Single depth frame.  Returns (x, y, z)
        in body frame (+X forward, +Y left, +Z up, origin at camera mount)
        for the centroid, or None if the depth frame is empty / the
        table plane is undetectable / no spike passes the filters.
        """
        table_z = self.table_plane_z()
        if table_z is None:
            return None
        frame = self.latest()
        if frame.depth_m is None or frame.depth_intrinsics is None:
            return None

        depth = frame.depth_m
        h, w = depth.shape
        fx = float(frame.depth_intrinsics["fx"])
        fy = float(frame.depth_intrinsics["fy"])
        cx0 = float(frame.depth_intrinsics["cx"])
        cy0 = float(frame.depth_intrinsics["cy"])

        us = np.arange(0, w, sample_stride, dtype=np.float32)
        vs = np.arange(0, h, sample_stride, dtype=np.float32)
        uu, vv = np.meshgrid(us, vs)
        dd = depth[vv.astype(np.int32), uu.astype(np.int32)]

        valid = (dd > 0.0) & (dd < self._DEPTH_MAX_M)
        if valid.sum() < 16:
            return None

        # Vectorised back-projection of every grid pixel into body frame.
        cam_x = (uu - cx0) * dd / fx
        cam_y = (vv - cy0) * dd / fy
        cam_z = dd
        t = math.radians(self._tilt_deg)
        ct, st = math.cos(t), math.sin(t)
        body_x = -cam_y * st + cam_z * ct
        body_y = -cam_x
        body_z = -cam_y * ct - cam_z * st

        x_lo, x_hi = float(x_crop_m[0]), float(x_crop_m[1])
        y_lo, y_hi = float(y_crop_m[0]), float(y_crop_m[1])
        mask = (
            valid
            & (body_z > (table_z + min_height_above_table_m))
            & (body_z <= self._BODY_Z_MAX_M)
            & (body_x >= x_lo) & (body_x <= x_hi)
            & (body_y >= y_lo) & (body_y <= y_hi)
        )
        if mask.sum() < 20:
            return None

        px = body_x[mask]
        py = body_y[mask]
        pz = body_z[mask]

        # 2D occupancy grid in (x, y).
        cell = self._OBJECT_GRID_CELL_M
        nx = max(1, int(math.ceil((x_hi - x_lo) / cell)))
        ny = max(1, int(math.ceil((y_hi - y_lo) / cell)))
        bx_idx = np.clip(np.floor((px - x_lo) / cell).astype(np.int32), 0, nx - 1)
        by_idx = np.clip(np.floor((py - y_lo) / cell).astype(np.int32), 0, ny - 1)
        occ_count = np.zeros((nx, ny), dtype=np.int32)
        np.add.at(occ_count, (bx_idx, by_idx), 1)
        binary = occ_count >= self._OBJECT_MIN_CELL_POINTS
        if not binary.any():
            return None

        labels, nlabels = _connected_components_2d(binary)
        if nlabels == 0:
            return None

        point_labels = labels[bx_idx, by_idx]
        center_x = 0.5 * (x_lo + x_hi)

        best = None
        best_score = None
        for lbl in range(1, nlabels + 1):
            sel = point_labels == lbl
            n = int(sel.sum())
            if n == 0:
                continue
            # Ignore blobs smaller than the minimum cell count (but count
            # CELLS, not points — a compact blob can have many points in
            # a single cell and still be noise).
            if int((labels == lbl).sum()) < self._OBJECT_MIN_CELLS:
                continue
            mx = float(px[sel].mean())
            my = float(py[sel].mean())
            mz = float(pz[sel].mean())
            # Score: (distance² to centre, x).  Lower is better.
            score = ((mx - center_x) ** 2 + my * my, mx)
            if best_score is None or score < best_score:
                best_score = score
                best = (mx, my, mz)
        return best

    def pixel_to_table_xy(self, u: int, v: int,
                          table_height_m: float = 0.74) -> Optional[np.ndarray]:
        """Return (x_forward, y_left) of a DEPTH pixel on a table plane.

        Uses the pixel's actual depth if valid; otherwise intersects the
        pixel ray with the plane z = -table_height_m in body frame.
        """
        body = self.pixel_to_body_xyz(u, v)
        if body is not None:
            return body[:2]

        # Ray-plane fallback.
        frame = self.latest()
        if frame.depth_intrinsics is None:
            return None
        fx = frame.depth_intrinsics["fx"]
        fy = frame.depth_intrinsics["fy"]
        cx0 = frame.depth_intrinsics["cx"]
        cy0 = frame.depth_intrinsics["cy"]
        rx = (u - cx0) / fx
        ry = (v - cy0) / fy
        # Apply the same rotation as pixel_to_body_xyz but to the unit ray
        # direction (cx=rx, cy=ry, cz=1).  The intersection parameter s
        # scales the ray from the camera origin to the table plane.
        t = math.radians(self._tilt_deg)
        ct, st = math.cos(t), math.sin(t)
        body_dir_x = -ry * st + 1.0 * ct
        body_dir_y = -rx
        body_dir_z = -ry * ct - 1.0 * st
        if abs(body_dir_z) < 1e-6:
            return None
        # Table plane is at body_z = -table_height_m (below the camera mount).
        s = -table_height_m / body_dir_z
        if s <= 0:
            return None
        return np.array([body_dir_x * s, body_dir_y * s], dtype=np.float32)


_CHANNEL_FACTORY_INITED = False


# ── Connected-components labeller (pure-numpy fallback for scipy.ndimage) ────

def _connected_components_2d(binary: np.ndarray) -> tuple:
    """Label 4-connected components of a 2D bool array.

    Returns (labels, n_labels) where `labels` has the same shape as
    `binary`, with 0 for background cells and [1..n_labels] for the
    connected components.  Uses scipy.ndimage.label when available, and a
    small union-find fallback (sized for the reach-envelope
    occupancy grid, ~30x30 cells) when scipy is absent so mock-path
    tests still run off-robot.
    """
    try:
        from scipy.ndimage import label  # type: ignore
        return label(binary)
    except Exception:  # noqa: BLE001
        pass

    nx, ny = binary.shape
    labels = np.zeros((nx, ny), dtype=np.int32)
    parent: list = [0]
    def _find(a: int) -> int:
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    def _union(a: int, b: int):
        ra, rb = _find(a), _find(b)
        if ra == rb:
            return
        if ra < rb:
            parent[rb] = ra
        else:
            parent[ra] = rb
    next_label = 1
    for i in range(nx):
        for j in range(ny):
            if not binary[i, j]:
                continue
            left  = labels[i, j - 1] if j > 0 else 0
            above = labels[i - 1, j] if i > 0 else 0
            if left == 0 and above == 0:
                labels[i, j] = next_label
                parent.append(next_label)
                next_label += 1
            elif left != 0 and above == 0:
                labels[i, j] = left
            elif above != 0 and left == 0:
                labels[i, j] = above
            else:
                mn = min(left, above)
                labels[i, j] = mn
                _union(left, above)
    # Resolve roots and compact to 1..n.
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


# ── Mock implementation (no hardware, no DDS) ────────────────────────────────

class MockDepthCameraSight:
    """Drop-in replacement for DepthCameraSight without hardware or DDS.

    Synthesises a planar depth map at `table_height_m` below the camera,
    with an optional cylindrical object at caller-specified table-XY.
    RGB is a solid-colour placeholder (DINOv2 features off it are
    meaningless — use a real image for vision tests).
    """

    def __init__(self,
                 width: int = DEFAULT_WIDTH,
                 height: int = DEFAULT_HEIGHT,
                 tilt_deg: float = CAMERA_TILT_DEG_DEFAULT,
                 fx: float = 600.0,
                 fy: float = 600.0,
                 table_height_m: float = 0.74,
                 object_xy_m: Optional[tuple[float, float]] = None,
                 object_radius_m: float = 0.04,
                 object_height_m: float = 0.2):
        self._w = width
        self._h = height
        self._tilt_deg = tilt_deg
        self._table_height_m = table_height_m
        self._object_xy_m = object_xy_m
        self._object_radius_m = object_radius_m
        self._object_height_m = object_height_m
        self._depth_intr = {
            "fx": fx, "fy": fy,
            "cx": width / 2.0, "cy": height / 2.0,
            "width": width, "height": height,
            "depth_scale": 1.0,
        }
        self._rgb_intr = dict(self._depth_intr)
        self._rgb_intr.pop("depth_scale", None)

    def set_object(self, xy_m: Optional[tuple[float, float]]):
        self._object_xy_m = xy_m

    def _synth_depth(self) -> np.ndarray:
        h, w = self._h, self._w
        fx = self._depth_intr["fx"]; fy = self._depth_intr["fy"]
        cx = self._depth_intr["cx"]; cy = self._depth_intr["cy"]
        us = np.arange(w, dtype=np.float32)[None, :].repeat(h, axis=0)
        vs = np.arange(h, dtype=np.float32)[:, None].repeat(w, axis=1)
        rx = (us - cx) / fx
        ry = (vs - cy) / fy
        t = math.radians(self._tilt_deg)
        ct, st = math.cos(t), math.sin(t)
        # Same rotation as pixel_to_body_xyz applied to the unit ray
        # direction (cam = (rx, ry, 1)).  The table plane is at body_z =
        # -table_height_m (below the camera mount), so a ray intersects
        # it when body_dir_z·s = -h, with s > 0.
        body_dir_x = -ry * st + 1.0 * ct
        body_dir_y = -rx
        body_dir_z = -ry * ct - 1.0 * st
        depth = np.where(
            body_dir_z < -1e-6,
            -self._table_height_m / body_dir_z,
            0.0,
        ).astype(np.float32)
        if self._object_xy_m is not None:
            ox, oy = self._object_xy_m
            bx = body_dir_x * depth
            by = body_dir_y * depth
            dist = np.sqrt((bx - ox) ** 2 + (by - oy) ** 2)
            on_obj = (dist < self._object_radius_m) & (depth > 0)
            shrink = 1.0 - (self._object_height_m / max(self._table_height_m, 1e-3))
            depth = np.where(on_obj, depth * shrink, depth).astype(np.float32)
        return depth

    def latest(self) -> SightFrame:
        depth = self._synth_depth()
        rgb = np.full((self._h, self._w, 3), fill_value=(60, 60, 80), dtype=np.uint8)
        now = time.monotonic()
        return SightFrame(
            rgb=rgb,
            depth_m=depth,
            rgb_timestamp=now,
            depth_timestamp=now,
            rgb_intrinsics=self._rgb_intr,
            depth_intrinsics=self._depth_intr,
            tilt_deg=self._tilt_deg,
        )

    def pixel_to_camera_xyz(self, u, v, depth_m_override=None):
        f = self.latest()
        d = float(depth_m_override) if depth_m_override is not None else float(f.depth_m[v, u])
        if d <= 0.0:
            return None
        fx = self._depth_intr["fx"]; fy = self._depth_intr["fy"]
        cx = self._depth_intr["cx"]; cy = self._depth_intr["cy"]
        return np.array([(u - cx) * d / fx, (v - cy) * d / fy, d], dtype=np.float32)

    def pixel_to_body_xyz(self, u, v, depth_m_override=None):
        cam = self.pixel_to_camera_xyz(u, v, depth_m_override)
        if cam is None:
            return None
        cx_, cy_, cz_ = float(cam[0]), float(cam[1]), float(cam[2])
        t = math.radians(self._tilt_deg)
        ct, st = math.cos(t), math.sin(t)
        return np.array([-cy_ * st + cz_ * ct, -cx_, -cy_ * ct - cz_ * st], dtype=np.float32)

    def pixel_to_table_xy(self, u, v, table_height_m=0.74):
        body = self.pixel_to_body_xyz(u, v)
        if body is None:
            return None
        return body[:2]

    def table_plane_z(self, sample_stride=8, valid_frac_min=0.2):
        """Mock table plane: the synthetic plane is at exactly
        `-self._table_height_m` by construction.  The cylindrical object
        (when present) raises a small patch but the median of the full
        grid is still the plane Z, matching the real camera's median
        behaviour.  Returns a float, never None — the mock always has a
        well-formed depth frame.
        """
        return float(-self._table_height_m)

    def table_plane_xyz(self, sample_stride=8, valid_frac_min=0.2,
                        bin_size_m=0.02, min_support_frac=0.03,
                        z_tolerance_m=0.04):
        """Mock counterpart to DepthCameraSight.table_plane_xyz.

        The synthetic depth scene has no table-XY concept — the plane
        fills the entire grid — so the centroid collapses to a
        constant representative point.  Returns `(plane_z, 0.35, 0.0)`
        which sits inside a typical approach-loop happy-path window
        (0.20 <= x <= 0.40 m, `|y| <= 0.15 m`) so mock E2E runs accept
        on first probe.  For edge cases (off-centre table, step-closer
        flow) construct a real DepthCameraSight test fixture with a
        handcrafted depth frame.
        """
        return (float(-self._table_height_m), 0.35, 0.0)

    def object_xyz_on_table_m(
        self,
        min_height_above_table_m: float = 0.05,
        x_crop_m: tuple = (0.20, 0.55),
        y_crop_m: tuple = (-0.30, 0.30),
        sample_stride: int = 4,
    ) -> Optional[tuple]:
        """Mock counterpart to the real camera primitive.

        Bypasses the grid-and-label path and reads `self._object_xy_m`
        directly — the mock already knows where the synthetic cylinder
        sits.  Returns None if no object is set OR the requested
        min-height clearance would swallow the object top.  Honours the
        crop parameters so a test that places an object outside the
        reach envelope correctly sees None (which exercises the
        fall-back-to-blind-sweep path in the caller).
        """
        if self._object_xy_m is None:
            return None
        ox, oy = float(self._object_xy_m[0]), float(self._object_xy_m[1])
        if not (x_crop_m[0] <= ox <= x_crop_m[1]):
            return None
        if not (y_crop_m[0] <= oy <= y_crop_m[1]):
            return None
        # Top of the cylinder is `object_height_m` above the table plane.
        obj_top_z = -self._table_height_m + float(self._object_height_m)
        if float(self._object_height_m) < min_height_above_table_m:
            return None
        return (ox, oy, obj_top_z)

    def shutdown(self):
        pass


# ── CLI diagnostic ───────────────────────────────────────────────────────────

def _diag_main():
    import argparse
    p = argparse.ArgumentParser(description="Depth Camera Sight diagnostic")
    p.add_argument("--mock", action="store_true")
    p.add_argument("--width",  type=int, default=DEFAULT_WIDTH)
    p.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    p.add_argument("--fps",    type=int, default=DEFAULT_FPS)
    p.add_argument("--tilt",   type=float, default=CAMERA_TILT_DEG_DEFAULT)
    p.add_argument("--frames", type=int, default=10)
    p.add_argument("--no-rgb", action="store_true",
                   help="Skip the Unitree DDS RGB path (depth-only)")
    p.add_argument("--no-depth", action="store_true",
                   help="Skip the librealsense depth path (rgb-only)")
    args = p.parse_args()

    if args.mock:
        cam = MockDepthCameraSight(width=args.width, height=args.height,
                                    tilt_deg=args.tilt,
                                    object_xy_m=(0.4, 0.0))
    else:
        cam = DepthCameraSight.instance(
            width=args.width, height=args.height, fps=args.fps,
            tilt_deg=args.tilt,
            enable_rgb=not args.no_rgb,
            enable_depth=not args.no_depth,
        )

    for i in range(args.frames):
        f = cam.latest()
        rgb_str = "None" if f.rgb is None else f"{f.rgb.shape}"
        if f.depth_m is None:
            dm_str = "None"
        else:
            v = int((f.depth_m > 0).sum())
            mn = float(f.depth_m[f.depth_m > 0].min()) if v else 0.0
            mx = float(f.depth_m.max())
            dm_str = f"{f.depth_m.shape} valid={v} min={mn:.3f}m max={mx:.3f}m"
        print(f"[{i}] rgb={rgb_str} depth={dm_str} "
              f"tilt={f.tilt_deg:.1f}° "
              f"rgb_ts={f.rgb_timestamp:.2f} depth_ts={f.depth_timestamp:.2f}")
        time.sleep(0.1)

    uc = args.width // 2
    vc = args.height // 2
    p3 = cam.pixel_to_body_xyz(uc, vc)
    print(f"pixel_to_body_xyz({uc},{vc}) = {p3}")
    pt = cam.pixel_to_table_xy(uc, vc)
    print(f"pixel_to_table_xy({uc},{vc}) = {pt}")

    if not args.mock:
        f = cam.latest()
        print(f"rgb_intrinsics:   {f.rgb_intrinsics}")
        print(f"depth_intrinsics: {f.depth_intrinsics}")

    cam.shutdown()


if __name__ == "__main__":
    _diag_main()
