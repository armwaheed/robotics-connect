#!/usr/bin/env python3
"""
Depth-based vision encoder for ACT-lite v2.

Replaces the DINOv2 RGB path after operator feedback: the z-map technique
(back-projecting depth into body frame, clustering above
the table plane) proved reliable on hardware, while pixel-level RGB via
DINOv2 was weak for this task and expensive on the Jetson CPU.  This
module produces a 72-dim vision feature from a single depth frame:

  8-dim STRUCTURED block
      [0] valid_flag          1.0 if an object was detected on the table
      [1] object_x_body_m     body-frame X of the object centroid (0 if invalid)
      [2] object_y_body_m     body-frame Y of the object centroid
      [3] object_z_body_m     body-frame Z of the object centroid (~ top)
      [4] table_z_body_m      body-frame Z of the table plane (~ -0.50 from
                              the G1's calibrated camera mount)
      [5] height_above_table  obj_z - table_z (0 if invalid)
      [6] depth_valid_frac    fraction of depth pixels that are in-range
                              (sanity channel — collapses to small values
                              when the depth sensor is obscured / failing)
      [7] y_offset_centre     |object_y|, a lateral-offset channel that
                              helps retrieval key off how far off-centre
                              the object is even when the y sign doesn't
                              matter.  0 if invalid.

  64-dim PATCH block
      8x8 heightmap (body_z - table_z) over a 30 cm x 30 cm window
      centred on object_xy (or the crop centre if no object).  Each cell
      stores the MAX height-above-table of all depth pixels that
      back-project into that cell.  Empty cells → 0.  Flatten row-major.

Typical block magnitudes (on a single bottle scene):
  structured |f| ≈ 0.8
  patch      |f| ≈ 0.3-0.5
  combined   |f| ≈ 1.0
The 30-dim normalised proprio block has |f| ≈ 1.17 — so vision and
proprio contribute comparably to cosine similarity with `_VISION_SCALE
= 1.0`, no renormalisation hack needed.

Runs in ~5 ms on the Jetson Cortex-A78AE cores (pure vectorised numpy:
one back-projection pass, one fill-bin pass, one connected-components
pass over a 32x32-ish occupancy grid).  Well under the 100 ms control
tick so it can run INLINE on the hot path — no background worker is
required.  `VisionCache` keeps the async pattern for parity with the
DINOv2 era, but it no longer buys latency.

Parameters that must stay in sync with the object-localisation path in
`depth_camera_sight.py` — both modules back-project the SAME depth crop
to agree on where "the object" is:

  X_CROP_M         = (0.20, 0.55)  # body-frame X forward extent
  Y_CROP_M         = (-0.30, 0.30) # body-frame Y lateral extent
  MIN_OBJ_HEIGHT_M = 0.05          # above-table cutoff for object pixels

If you change these here, change them in the corresponding places in
`depth_camera_sight.py` so the two modules see the same object.
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np


VISION_DIM      = 72
STRUCTURED_DIM  = 8
PATCH_N         = 8
PATCH_DIM       = PATCH_N * PATCH_N     # 64
assert STRUCTURED_DIM + PATCH_DIM == VISION_DIM

# Depth range.  Pixels outside (0, DEPTH_MAX_M) are treated as invalid.
DEPTH_MAX_M = 4.0

# Back-projection crop in body frame (forward X, lateral Y).  Matches
# object_xyz_on_table_m so both consumers see the
# same "workspace".
X_CROP_M = (0.20, 0.55)
Y_CROP_M = (-0.30, 0.30)

# Minimum object height above the table plane to count as a spike.  5 cm
# covers a water bottle but filters out table-surface noise.
MIN_OBJ_HEIGHT_M = 0.05

# Pixel sampling stride over the raw depth image during back-projection.
# Stride 4 on 640x480 depth → 160x120 = 19200 samples — plenty for an
# 8x8 patch over a 30 cm window.  Larger stride = faster encode but noisier
# patch; smaller = slower.
SAMPLE_STRIDE = 4

# Local heightmap window — 30 cm x 30 cm around the object.  A bit wider
# than the object itself so the patch has some "table context" around it
# (retrieval should be able to tell "bottle on a sparse table" from
# "bottle in a cluttered scene", even if clutter detection is a future
# nice-to-have).
PATCH_SIZE_M = 0.30

# Fallback patch centre when no object is detected.  Matches the middle
# of the body-frame crop window above.
_CROP_CENTER_X = 0.5 * (X_CROP_M[0] + X_CROP_M[1])  # 0.375
_CROP_CENTER_Y = 0.5 * (Y_CROP_M[0] + Y_CROP_M[1])  # 0.000

# Occupancy-grid cell size for connected-components (same as the
# object-localisation path for consistency).  2.5 cm cells → ~14x24 grid over the full crop, which
# is plenty for one object.
OBJECT_GRID_CELL_M  = 0.025
OBJECT_MIN_CELLS    = 3    # ≥3 cells (≈ 1.8 cm² footprint) to count as an object


def _backproject_to_body(depth_m: np.ndarray,
                         intrinsics: dict,
                         tilt_deg: float,
                         stride: int = SAMPLE_STRIDE):
    """Vectorised back-projection of a depth image into body frame.

    Returns a tuple (body_x, body_y, body_z, valid) where each array is
    the flattened per-sample result over the strided grid of depth
    pixels.  Body frame convention: +X forward, +Y left, +Z up, origin
    at the camera mount.  Matches DepthCameraSight.object_xyz_on_table_m.
    """
    fx = float(intrinsics["fx"])
    fy = float(intrinsics["fy"])
    cx0 = float(intrinsics["cx"])
    cy0 = float(intrinsics["cy"])
    h, w = depth_m.shape
    us = np.arange(0, w, stride, dtype=np.float32)
    vs = np.arange(0, h, stride, dtype=np.float32)
    uu, vv = np.meshgrid(us, vs)
    dd = depth_m[vv.astype(np.int32), uu.astype(np.int32)]
    valid = (dd > 0.0) & (dd < DEPTH_MAX_M)

    cam_x = (uu - cx0) * dd / fx
    cam_y = (vv - cy0) * dd / fy
    cam_z = dd
    t = math.radians(float(tilt_deg))
    ct, st = math.cos(t), math.sin(t)
    body_x = -cam_y * st + cam_z * ct
    body_y = -cam_x
    body_z = -cam_y * ct - cam_z * st
    return body_x.ravel(), body_y.ravel(), body_z.ravel(), valid.ravel()


def _table_plane_z_from_samples(body_z: np.ndarray,
                                valid: np.ndarray,
                                body_x: np.ndarray,
                                body_y: np.ndarray) -> Optional[float]:
    """Estimate the table plane Z from back-projected samples.

    Median of in-crop valid body_z values.  Matches the "median of a
    coarse grid over the depth image" heuristic that DepthCameraSight
    uses for table_plane_z(), but applied to the SAME crop we care about
    for the object detector.  Returns None when too few samples fall
    in-crop.
    """
    mask = (
        valid
        & (body_x >= X_CROP_M[0]) & (body_x <= X_CROP_M[1])
        & (body_y >= Y_CROP_M[0]) & (body_y <= Y_CROP_M[1])
    )
    if int(mask.sum()) < 64:
        return None
    return float(np.median(body_z[mask]))


# ── Connected-components (minimal reimplementation; matches the object path) ─

def _cc_2d(binary: np.ndarray):
    try:
        from scipy.ndimage import label  # type: ignore
        labels, n = label(binary)
        return labels.astype(np.int32), int(n)
    except Exception:  # noqa: BLE001
        return _cc_2d_fallback(binary)


def _cc_2d_fallback(binary: np.ndarray):
    nx, ny = binary.shape
    labels = np.zeros((nx, ny), dtype=np.int32)
    parent: list[int] = [0]

    def _find(a: int) -> int:
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def _union(a: int, b: int):
        ra, rb = _find(a), _find(b)
        if ra != rb:
            parent[ra if ra < rb else rb] = ra if ra > rb else rb
            parent[rb if ra < rb else ra] = ra if ra < rb else rb

    next_label = 0
    for i in range(nx):
        for j in range(ny):
            if not binary[i, j]:
                continue
            up    = labels[i - 1, j] if i > 0 else 0
            left  = labels[i, j - 1] if j > 0 else 0
            if up == 0 and left == 0:
                next_label += 1
                parent.append(next_label)
                labels[i, j] = next_label
            elif up != 0 and left == 0:
                labels[i, j] = up
            elif up == 0 and left != 0:
                labels[i, j] = left
            else:
                labels[i, j] = up
                _union(up, left)
    remap: dict[int, int] = {}
    out_next = 0
    for i in range(nx):
        for j in range(ny):
            lbl = labels[i, j]
            if lbl == 0:
                continue
            root = _find(lbl)
            if root not in remap:
                out_next += 1
                remap[root] = out_next
            labels[i, j] = remap[root]
    return labels, out_next


def _object_xy_from_samples(body_x: np.ndarray, body_y: np.ndarray,
                            body_z: np.ndarray, valid: np.ndarray,
                            table_z: float) -> Optional[tuple]:
    """Return (ox, oy, oz) body-frame centroid of the largest above-table
    blob in the crop, or None.  Same algorithm as
    DepthCameraSight.object_xyz_on_table_m but on the flattened samples
    we already back-projected, so the cost is one grid-and-label pass.
    """
    mask = (
        valid
        & (body_z > (table_z + MIN_OBJ_HEIGHT_M))
        & (body_x >= X_CROP_M[0]) & (body_x <= X_CROP_M[1])
        & (body_y >= Y_CROP_M[0]) & (body_y <= Y_CROP_M[1])
    )
    if int(mask.sum()) < 20:
        return None

    px = body_x[mask]
    py = body_y[mask]
    pz = body_z[mask]

    x_lo, x_hi = X_CROP_M
    y_lo, y_hi = Y_CROP_M
    cell = OBJECT_GRID_CELL_M
    nx = max(1, int(math.ceil((x_hi - x_lo) / cell)))
    ny = max(1, int(math.ceil((y_hi - y_lo) / cell)))
    bx_idx = np.clip(np.floor((px - x_lo) / cell).astype(np.int32), 0, nx - 1)
    by_idx = np.clip(np.floor((py - y_lo) / cell).astype(np.int32), 0, ny - 1)
    occ = np.zeros((nx, ny), dtype=np.int32)
    np.add.at(occ, (bx_idx, by_idx), 1)
    binary = occ >= 1
    if not binary.any():
        return None

    labels, nlabels = _cc_2d(binary)
    if nlabels == 0:
        return None

    point_labels = labels[bx_idx, by_idx]
    best = None
    best_score = None
    centre_x = 0.5 * (x_lo + x_hi)
    for lbl in range(1, nlabels + 1):
        if int((labels == lbl).sum()) < OBJECT_MIN_CELLS:
            continue
        sel = point_labels == lbl
        if int(sel.sum()) == 0:
            continue
        mx = float(px[sel].mean())
        my = float(py[sel].mean())
        mz = float(pz[sel].mean())
        score = ((mx - centre_x) ** 2 + my * my, mx)
        if best_score is None or score < best_score:
            best_score = score
            best = (mx, my, mz)
    return best


def _heightmap_patch(body_x: np.ndarray, body_y: np.ndarray,
                     body_z: np.ndarray, valid: np.ndarray,
                     table_z: float,
                     centre_xy: Tuple[float, float],
                     patch_n: int = PATCH_N,
                     patch_size_m: float = PATCH_SIZE_M) -> np.ndarray:
    """Build a (patch_n, patch_n) heightmap centred on centre_xy.

    Each cell holds the MAX (body_z - table_z) of all valid samples
    that back-project inside the cell's (x, y) bin.  Empty cells → 0.
    Max (rather than mean) keeps the spike that says "there's an object
    sticking up here" visible in the patch even if most samples in the
    cell land on the table behind the object.
    """
    cx, cy = centre_xy
    half = 0.5 * patch_size_m
    x_lo, x_hi = cx - half, cx + half
    y_lo, y_hi = cy - half, cy + half
    cell = patch_size_m / patch_n

    mask = (
        valid
        & (body_x >= x_lo) & (body_x < x_hi)
        & (body_y >= y_lo) & (body_y < y_hi)
    )
    if not mask.any():
        return np.zeros((patch_n, patch_n), dtype=np.float32)

    bx_idx = np.clip(np.floor((body_x[mask] - x_lo) / cell).astype(np.int32),
                     0, patch_n - 1)
    by_idx = np.clip(np.floor((body_y[mask] - y_lo) / cell).astype(np.int32),
                     0, patch_n - 1)
    h = (body_z[mask] - table_z).astype(np.float32)

    patch = np.zeros((patch_n, patch_n), dtype=np.float32)
    # np.maximum.at is unbuffered reduction — keeps the tallest sample
    # per cell without needing a manual groupby.
    np.maximum.at(patch, (bx_idx, by_idx), h)
    # Don't let negative heights (depth noise below the table plane)
    # leak in — they're almost always artefacts and mess with the
    # cosine direction.
    return np.clip(patch, 0.0, None)


def encode_vision_depth(frame) -> np.ndarray:
    """Encode a SightFrame into a 72-dim depth vision feature.

    `frame` must duck-type SightFrame: attributes `depth_m` (float32 (H,W)
    array, 0 = invalid), `depth_intrinsics` (dict with fx, fy, cx, cy),
    and `tilt_deg` (float).  Returns all-zeros on any missing input so
    rebuild / live can both feed partial frames without raising.
    """
    feat = np.zeros(VISION_DIM, dtype=np.float32)
    depth_m = getattr(frame, "depth_m", None)
    intr    = getattr(frame, "depth_intrinsics", None)
    tilt    = getattr(frame, "tilt_deg", None)
    if depth_m is None or intr is None or tilt is None:
        return feat

    body_x, body_y, body_z, valid = _backproject_to_body(
        depth_m, intr, tilt, stride=SAMPLE_STRIDE,
    )
    total_samples = body_x.shape[0]
    valid_frac = float(valid.sum()) / float(max(total_samples, 1))

    table_z = _table_plane_z_from_samples(body_z, valid, body_x, body_y)
    if table_z is None:
        # Depth sensor can't even see the table — emit the valid_frac
        # channel so retrieval can at least distinguish "sensor failure"
        # from "object not present".
        feat[6] = valid_frac
        return feat

    obj = _object_xy_from_samples(body_x, body_y, body_z, valid, table_z)
    if obj is not None:
        ox, oy, oz = obj
        feat[0] = 1.0                         # valid_flag
        feat[1] = float(ox)                   # object_x
        feat[2] = float(oy)                   # object_y
        feat[3] = float(oz)                   # object_z
        feat[4] = float(table_z)              # table_z
        feat[5] = float(oz - table_z)         # height_above_table
        feat[6] = valid_frac                  # depth_valid_frac
        feat[7] = float(abs(oy))              # |y_offset|
        patch_centre = (ox, oy)
    else:
        # No object detected.  Structured block: only the valid channels
        # — valid_flag stays 0 so retrieval keys off the "no object"
        # state correctly, and table_z + valid_frac are still informative.
        feat[4] = float(table_z)
        feat[6] = valid_frac
        patch_centre = (_CROP_CENTER_X, _CROP_CENTER_Y)

    patch = _heightmap_patch(
        body_x, body_y, body_z, valid, table_z, patch_centre,
    )
    feat[STRUCTURED_DIM:] = patch.reshape(-1)
    return feat


# ── Stored-frame shim — for rebuild_features.py offline use ──────────────────

class StoredSightFrame:
    """Minimal duck-typed SightFrame for rebuild_features.py.

    Takes the uint16-millimetres depth array out of the .npz, reinflates
    it to float32 metres, and provides the three attributes
    encode_vision_depth needs.  Keeps rebuild independent of the live
    camera class.
    """
    __slots__ = ("depth_m", "depth_intrinsics", "tilt_deg")

    def __init__(self, depth_u16_mm: np.ndarray, intrinsics: dict,
                 tilt_deg: float):
        if depth_u16_mm.dtype != np.uint16:
            raise ValueError(
                f"StoredSightFrame: depth dtype {depth_u16_mm.dtype} != uint16"
            )
        self.depth_m = depth_u16_mm.astype(np.float32) * 0.001
        self.depth_intrinsics = dict(intrinsics)
        self.tilt_deg = float(tilt_deg)


def depth_m_to_u16_mm(depth_m: np.ndarray) -> np.ndarray:
    """Convert float32 depth-in-metres to uint16 millimetres for storage.

    Clamps to [0, 65535] mm (65 m upper bound — well past DEPTH_MAX_M).
    Invalid pixels (0.0 m) stay at 0.
    """
    dm = np.asarray(depth_m, dtype=np.float32)
    mm = np.rint(dm * 1000.0)
    mm = np.clip(mm, 0, 65535).astype(np.uint16)
    return mm
