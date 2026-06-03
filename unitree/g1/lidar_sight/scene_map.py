#!/usr/bin/env python3
"""
scene_map — 2-D top-down occupancy grid + goal picker + A* planner for
a navigation caller.

SceneMap turns one `LidarCloud` + the list of detected `Table`s into a
coarse (default 10 cm) body-frame grid whose cells carry one of four
states:

    FREE      — no LiDAR returns in the navigable z-band at this (x, y)
    OBSTACLE  — at least one non-table LiDAR return in the navigable band
    TABLE     — at least one LiDAR return inside a detected table's footprint
    UNKNOWN   — no LiDAR returns at all for this (x, y) column

"Navigable band" = vertical window the robot body sweeps while walking:
slightly above the floor (to reject noise hugging the ground) up to
slightly above the torso (above which overhead lights / ceiling do not
block walking).  The band is derived from the cloud itself: floor_z is
estimated from the cloud's densest low-z bin, and the top of the band
sits a fixed `NAV_CEILING_ABOVE_FLOOR_M` above it.

`pick_goal_table` is the one-shot goal selector invoked at
goal-selection time.  "nearest_prefer_heading" picks the table closest
by Euclidean distance with a soft preference for tables along body +X.

`plan_path` is an A* grid search with an inflation radius equal to the
robot's half-width + a safety margin.  UNKNOWN cells are treated as
OBSTACLE by default (fail-safe), configurable.

Kept intentionally crude — no probability fusion, no SLAM, no dynamic
object tracking.  Good enough for a 3 m walk through a living-room-scale
obstacle course, which is the intended scope.
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

# scene_map is in the same directory as lidar_sight; callers add this dir
# to sys.path before importing either, so a relative import works as a
# flat-module reference.
from lidar_sight import LidarCloud, Table  # type: ignore  # noqa: E402


# ── Cell labels ──────────────────────────────────────────────────────────────

FREE: int = 0
OBSTACLE: int = 1
TABLE: int = 2
UNKNOWN: int = 3


# ── Default geometry (tuned for G1 EDU in a living-room-scale scene) ─────────

DEFAULT_CELL_M = 0.10              # 10 cm cells — crude but fast
DEFAULT_X_BOUNDS = (-1.0, 5.0)     # body-frame forward window (metres)
DEFAULT_Y_BOUNDS = (-3.0, 3.0)     # body-frame lateral window (metres)

# Navigable z-band: points within this band (body frame, metres) count as
# blocking obstacles.  Anything below floor_offset is floor noise; anything
# above the ceiling offset is overhead and irrelevant to walking.
NAV_FLOOR_OFFSET_M   = 0.05        # drop points within 5 cm of the floor
NAV_CEILING_ABOVE_FLOOR_M = 1.35   # above floor; covers the G1's full height

# Fallback floor estimate when the cloud is too sparse or too occluded to
# find a dense low-z bin.  The standing-Regular-Mode G1 torso_link sits
# ~0.80 m above the floor (see lidar_sight README calibration), so the
# floor is ~-0.80 m in body frame.
FLOOR_FALLBACK_BODY_Z_M = -0.80

# Default robot clearance for path inflation.  The G1 EDU's widest walking
# footprint is ~0.35 m across (≈0.175 m half-width); we inflate obstacles
# by half-width + a small margin.  Live-tested against the dense obstacle
# course: 0.35 m was too conservative (ringed the robot with inflated
# blockage and walled in every outbound path), 0.22 m consistently finds
# a clean path through gaps ≥ 0.44 m.
DEFAULT_CLEARANCE_M = 0.22


# ── SceneMap ────────────────────────────────────────────────────────────────

@dataclass
class SceneMap:
    """Top-down occupancy grid in body frame.

    `grid[i, j]` corresponds to the cell whose x-centre is
    `x_bounds[0] + (i + 0.5) * cell_m` and whose y-centre is
    `y_bounds[0] + (j + 0.5) * cell_m`.  Axis 0 is forward (body +X),
    axis 1 is lateral (body +Y, left).
    """
    grid: np.ndarray           # (nx, ny) int8
    x_bounds: Tuple[float, float]
    y_bounds: Tuple[float, float]
    cell_m: float
    floor_z: float             # body-frame z used for the navigable-band floor
    nav_z_min: float           # lower edge of the navigable band
    nav_z_max: float           # upper edge of the navigable band
    tables: Tuple[Table, ...]  # tables the map was built against (pass-through)

    # Convenience ----------------------------------------------------------

    @property
    def shape(self) -> Tuple[int, int]:
        return tuple(self.grid.shape)  # type: ignore[return-value]

    def cell_to_xy(self, i: int, j: int) -> Tuple[float, float]:
        return (
            float(self.x_bounds[0] + (i + 0.5) * self.cell_m),
            float(self.y_bounds[0] + (j + 0.5) * self.cell_m),
        )

    def xy_to_cell(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        i = int(math.floor((x - self.x_bounds[0]) / self.cell_m))
        j = int(math.floor((y - self.y_bounds[0]) / self.cell_m))
        nx, ny = self.grid.shape
        if 0 <= i < nx and 0 <= j < ny:
            return i, j
        return None

    def count(self, label: int) -> int:
        return int((self.grid == label).sum())

    def summary(self) -> str:
        return (
            f"SceneMap {self.grid.shape}  "
            f"FREE={self.count(FREE)}  OBSTACLE={self.count(OBSTACLE)}  "
            f"TABLE={self.count(TABLE)}  UNKNOWN={self.count(UNKNOWN)}  "
            f"floor_z={self.floor_z:+.3f}  "
            f"nav_z=[{self.nav_z_min:+.3f}, {self.nav_z_max:+.3f}]"
        )


# ── Floor estimation (robust to chin occlusion) ─────────────────────────────

_FLOOR_HIST_BIN_M = 0.05
_FLOOR_MIN_SUPPORT = 300


def _estimate_floor_z(points: np.ndarray) -> float:
    """Densest z bin in the cloud (lower half), else the fallback.

    History: the previous implementation returned the *lowest* z-bin
    above the `_FLOOR_MIN_SUPPORT` count threshold.  In the 2026-05-13
    bistro-table run, the chin/face-frame produced a spurious low-z
    spike at z≈-1.25 (331 points, just above the 300-point support
    threshold), while the *real* floor cluster at z≈-1.05 had 3037
    points — 9× denser.  The old picker chose the spurious spike,
    shifting the entire nav band down by ~20 cm.  Real floor returns
    at z≈-0.9 to -1.0 then fell INSIDE the nav band and got
    classified as OBSTACLE, putting the robot itself onto a "red
    OBSTACLE" cell — which made every plan fail.

    New behaviour: pick the **densest** bin in the LOWER half of the
    z-distribution (capping above the body-origin so we never select
    a ceiling-height cluster as floor).  This is robust to a small
    number of below-floor reflections, and the lower-half cap keeps
    a dense ceiling return from being mistaken for floor in a tight
    room where the LiDAR sees more ceiling than floor.
    """
    if points.shape[0] < _FLOOR_MIN_SUPPORT:
        return FLOOR_FALLBACK_BODY_Z_M
    zs = points[:, 2]
    zs = zs[(zs > -2.0) & (zs < 2.0)]
    if zs.size < _FLOOR_MIN_SUPPORT:
        return FLOOR_FALLBACK_BODY_Z_M
    lo, hi = float(zs.min()), float(zs.max())
    nbins = max(4, int(math.ceil((hi - lo) / _FLOOR_HIST_BIN_M)))
    counts, edges = np.histogram(zs, bins=nbins)
    # Restrict the candidate range to the body-frame z-band where the
    # floor physically lives for a standing G1.  The mid-360 sits at
    # body z ≈ +0.42 m above the torso link, the torso is ~0.95 m
    # above the floor when standing, so the floor is at body z ≈
    # -0.95 ± stance variation.  Bounds:
    #   * upper cap z=-0.30: walls/desks/the LiDAR-self-reflection
    #     band all live above this; excludes them from floor candidacy.
    #   * lower cap z=-1.50: any return below this is a chin/face-rim
    #     reflection or a stairwell-leakage point; excludes them too.
    # Within that band, pick the DENSEST bin (mode) — the floor is by
    # far the densest single horizontal plane in the LiDAR's lower
    # hemisphere.
    centers = 0.5 * (edges[:-1] + edges[1:])
    in_band = (centers >= -1.50) & (centers <= -0.30)
    if not np.any(in_band):
        return FLOOR_FALLBACK_BODY_Z_M
    counts_band = counts.copy()
    counts_band[~in_band] = 0
    if counts_band.max() < _FLOOR_MIN_SUPPORT:
        return FLOOR_FALLBACK_BODY_Z_M
    best = int(np.argmax(counts_band))
    return float(centers[best])


# ── Scene map construction ──────────────────────────────────────────────────

def build_scene_map(cloud: LidarCloud,
                    tables: Sequence[Table],
                    x_bounds: Tuple[float, float] = DEFAULT_X_BOUNDS,
                    y_bounds: Tuple[float, float] = DEFAULT_Y_BOUNDS,
                    cell_m: float = DEFAULT_CELL_M,
                    floor_z_override: Optional[float] = None,
                    nav_ceiling_above_floor_m: float = NAV_CEILING_ABOVE_FLOOR_M,
                    table_margin_m: float = 0.05) -> SceneMap:
    """Build an occupancy grid from one LiDAR cloud and its detected tables.

    `table_margin_m` is the slack around each table's centroid used to
    define its TABLE footprint on the grid.  The footprint is a bounding
    box inferred from the in-band points that land inside the table's
    connected component (we re-find it here so we don't need the raw
    clustering output from lidar_sight).
    """
    points = cloud.points.astype(np.float32, copy=False)
    x_lo, x_hi = float(x_bounds[0]), float(x_bounds[1])
    y_lo, y_hi = float(y_bounds[0]), float(y_bounds[1])
    nx = max(1, int(math.ceil((x_hi - x_lo) / cell_m)))
    ny = max(1, int(math.ceil((y_hi - y_lo) / cell_m)))

    # 1) Floor estimate.
    floor_z = (
        float(floor_z_override)
        if floor_z_override is not None
        else _estimate_floor_z(points)
    )
    nav_z_min = floor_z + NAV_FLOOR_OFFSET_M
    nav_z_max = floor_z + nav_ceiling_above_floor_m

    # 2) Column occupancy — any point at all in this (x, y), plus any
    #    point in the navigable z-band.
    in_bounds = (
        (points[:, 0] >= x_lo) & (points[:, 0] < x_hi)
        & (points[:, 1] >= y_lo) & (points[:, 1] < y_hi)
    )
    if not np.any(in_bounds):
        # Totally empty scene — whole map is UNKNOWN.
        grid = np.full((nx, ny), UNKNOWN, dtype=np.int8)
        return SceneMap(
            grid=grid, x_bounds=x_bounds, y_bounds=y_bounds, cell_m=cell_m,
            floor_z=floor_z, nav_z_min=nav_z_min, nav_z_max=nav_z_max,
            tables=tuple(tables),
        )
    p = points[in_bounds]
    xi = np.clip(np.floor((p[:, 0] - x_lo) / cell_m).astype(np.int32), 0, nx - 1)
    yi = np.clip(np.floor((p[:, 1] - y_lo) / cell_m).astype(np.int32), 0, ny - 1)

    # Any-point-in-cell -> cell had LiDAR coverage at all.
    any_mask = np.zeros((nx, ny), dtype=bool)
    any_mask[xi, yi] = True

    # Band-point-in-cell -> cell has an obstacle.
    in_band = (p[:, 2] >= nav_z_min) & (p[:, 2] <= nav_z_max)
    if np.any(in_band):
        obs_xi = xi[in_band]
        obs_yi = yi[in_band]
        obs_mask = np.zeros((nx, ny), dtype=bool)
        obs_mask[obs_xi, obs_yi] = True
    else:
        obs_mask = np.zeros((nx, ny), dtype=bool)

    # 3) Tables — mark TABLE cells for every (x, y) inside a detected
    #    table's bounding box.  We estimate the bbox from the in-band
    #    points that fall within each table's z-slab + lateral footprint
    #    area — this re-derives the cluster here without reaching into
    #    lidar_sight internals.
    table_mask = np.zeros((nx, ny), dtype=bool)
    for t in tables:
        cx, cy, cz = t.center_xyz
        # Approximate the footprint as a square with side
        # sqrt(area_estimate_m2), plus a small margin.  Real tables are
        # rectangular but the connected-component step already squished
        # them into a bounding blob — a square of the right area is a
        # good enough stand-in for marking the map.
        side = math.sqrt(max(0.0, t.area_estimate_m2))
        half = side / 2.0 + table_margin_m
        # Convert to cell index ranges.
        i_lo = max(0, int(math.floor((cx - half - x_lo) / cell_m)))
        i_hi = min(nx, int(math.ceil((cx + half - x_lo) / cell_m)))
        j_lo = max(0, int(math.floor((cy - half - y_lo) / cell_m)))
        j_hi = min(ny, int(math.ceil((cy + half - y_lo) / cell_m)))
        if i_hi > i_lo and j_hi > j_lo:
            # AND with the cells that actually had points near this z —
            # keeps us from marking empty cells in the bounding box as
            # TABLE when the real table is smaller than the bbox.
            slab_mask = np.zeros((nx, ny), dtype=bool)
            z_lo = cz - 0.08
            z_hi = cz + 0.08
            sel = (p[:, 2] >= z_lo) & (p[:, 2] <= z_hi)
            if np.any(sel):
                slab_mask[xi[sel], yi[sel]] = True
            sub = np.zeros((nx, ny), dtype=bool)
            sub[i_lo:i_hi, j_lo:j_hi] = True
            table_mask |= (sub & slab_mask)

    # 4) Compose labels.  TABLE wins over OBSTACLE wins over FREE; cells
    #    with no coverage at all are UNKNOWN.
    grid = np.full((nx, ny), UNKNOWN, dtype=np.int8)
    grid[any_mask] = FREE
    grid[obs_mask] = OBSTACLE
    grid[table_mask] = TABLE

    return SceneMap(
        grid=grid,
        x_bounds=x_bounds, y_bounds=y_bounds, cell_m=cell_m,
        floor_z=floor_z, nav_z_min=nav_z_min, nav_z_max=nav_z_max,
        tables=tuple(tables),
    )


# ── Goal picker ─────────────────────────────────────────────────────────────
#
# Preference relation (bucketed, transitive: tables along-heading
# preferred over proximal; within either category, proximal preferred
# over distant):
#
#   Bucket 1 "on-path":  forward (body +X > 0) AND at reasonable range
#                         (d ≤ max_forward_m) AND within the heading cone
#                         (|angle from +X| ≤ heading_cone_rad).
#   Bucket 2 "off-path": everything else that survives the min-area gate.
#
# Within each bucket: nearest Euclidean wins.  Bucket 1 strictly preferred
# over Bucket 2.  If both buckets are empty (no eligible tables), return
# None.
#
# Dedup pass: several detections with near-identical xy but different z
# are often a single multi-level object (bookshelf) or a stack of self-
# reflections.  We collapse them to one representative per xy cluster so
# the bucketed picker doesn't see the cluster as five separate tables.


def dedup_tables(tables: Sequence[Table],
                 xy_merge_radius_m: float = 0.25) -> List[Table]:
    """Collapse detections whose xy centres are within `xy_merge_radius_m`
    into a single representative (the one with the most points).

    Kept deliberately simple — just greedy single-link clustering over
    the xy centres.  For the living-room-scale clutter this module is
    aimed at, merging within 25 cm is conservative enough that distinct
    real tables do not get fused.
    """
    if not tables:
        return []
    sorted_by_pts = sorted(tables, key=lambda t: -t.point_count)
    kept: List[Table] = []
    for cand in sorted_by_pts:
        cx, cy, _ = cand.center_xyz
        merged = False
        for k in kept:
            kx, ky, _ = k.center_xyz
            if math.hypot(cx - kx, cy - ky) <= xy_merge_radius_m:
                merged = True
                break
        if not merged:
            kept.append(cand)
    return kept


def pick_goal_table(tables: Sequence[Table],
                    strategy: str = "bucket_heading_then_nearest",
                    heading_cone_rad: float = math.pi / 6.0,   # ±30°
                    max_forward_m: float = 8.0,
                    min_area_m2: float = 0.05,
                    dedup_xy_radius_m: float = 0.25) -> Optional[Table]:
    """Select the single goal-state table at goal-selection time.

    `bucket_heading_then_nearest` (default): two-bucket lexicographic
    preference described in the module header.  Tables along the
    current heading axis strictly win over off-axis tables; within each
    bucket, nearest Euclidean wins.

    `nearest_prefer_heading` (legacy): single-score `distance +
    0.30 × |angle|`.  Retained so mock tests that predate the bucketed
    picker keep working.

    Both strategies:
      * Reject tables whose area < `min_area_m2`.
      * Deduplicate xy-colocated detections (same pile of shelves etc.).
    """
    eligible = [t for t in tables if t.area_estimate_m2 >= min_area_m2]
    if not eligible:
        return None
    eligible = dedup_tables(eligible, xy_merge_radius_m=dedup_xy_radius_m)
    if not eligible:
        return None

    if strategy == "bucket_heading_then_nearest":
        on_path: List[Table] = []
        off_path: List[Table] = []
        for t in eligible:
            x, y, _ = t.center_xyz
            d = math.hypot(x, y)
            if x <= 0 or d > max_forward_m:
                off_path.append(t)
                continue
            ang = abs(math.atan2(y, x))
            if ang <= heading_cone_rad:
                on_path.append(t)
            else:
                off_path.append(t)

        def _dist(t: Table) -> float:
            return math.hypot(t.center_xyz[0], t.center_xyz[1])

        if on_path:
            return min(on_path, key=_dist)
        return min(off_path, key=_dist)

    if strategy == "nearest_prefer_heading":
        heading_penalty_per_rad = 0.30

        def _score(t: Table) -> float:
            x, y, _ = t.center_xyz
            d = math.hypot(x, y)
            ang = abs(math.atan2(y, x))
            return float(d + heading_penalty_per_rad * ang)
        return min(eligible, key=_score)

    raise ValueError(f"Unknown pick_goal_table strategy: {strategy!r}")


# ── A* path planner ─────────────────────────────────────────────────────────

_NEIGHBOURS_8 = (
    (-1, -1, math.sqrt(2)), (-1, 0, 1.0), (-1, 1, math.sqrt(2)),
    ( 0, -1, 1.0),                        ( 0, 1, 1.0),
    ( 1, -1, math.sqrt(2)), ( 1, 0, 1.0), ( 1, 1, math.sqrt(2)),
)


def _inflate_obstacles(grid: np.ndarray,
                       cell_m: float,
                       clearance_m: float,
                       unknown_as_obstacle: bool) -> np.ndarray:
    """Return a bool mask of cells that the robot CANNOT occupy.

    Dilates the OBSTACLE mask (optionally merged with UNKNOWN) by
    Euclidean distance ≤ `clearance_m`.  TABLE cells are NOT treated
    as blocked for planning — the goal sits inside a table, so
    dilating away from tables would make every goal unreachable.  We
    handle table-adjacency in the goal-cell selection step instead.

    History: an earlier version used a *chessboard*-distance dilation.
    That treats every cell in a (2r+1)×(2r+1) square as inflated,
    which at cell_m=0.10 m and clearance_m=0.18 m (r=2 cells) gives
    an effective clearance of 2·√2·0.10 = 0.28 m at the diagonals —
    56% over the requested clearance.  With ~900 OBSTACLE cells in a
    cluttered home-office scene the chessboard dilation floods the
    corridor between robot and goal, leaving A* with only a trivial
    0.25 m path.  Euclidean-disc dilation costs the same per-frame
    (precomputed offset table) but only marks cells within the
    requested radius, opening the corridors back up.
    """
    r_cells_float = clearance_m / cell_m
    r = int(math.ceil(r_cells_float))
    blocked = grid == OBSTACLE
    if unknown_as_obstacle:
        blocked = blocked | (grid == UNKNOWN)
    if r <= 0:
        return blocked
    # Euclidean-disc dilation: a cell is blocked if any cell within
    # Euclidean distance `clearance_m` is blocked.  Implemented as an
    # OR over precomputed (di, dj) offsets that fall inside the disc
    # of radius r_cells_float.
    r2 = r_cells_float * r_cells_float
    nx, ny = blocked.shape
    out = blocked.copy()
    for di in range(-r, r + 1):
        for dj in range(-r, r + 1):
            if di == 0 and dj == 0:
                continue
            if di * di + dj * dj > r2:
                continue
            # Shift `blocked` by (di, dj) and OR into `out`.
            src_i_lo = max(0, -di)
            src_i_hi = min(nx, nx - di)
            src_j_lo = max(0, -dj)
            src_j_hi = min(ny, ny - dj)
            dst_i_lo = src_i_lo + di
            dst_i_hi = src_i_hi + di
            dst_j_lo = src_j_lo + dj
            dst_j_hi = src_j_hi + dj
            out[dst_i_lo:dst_i_hi, dst_j_lo:dst_j_hi] |= (
                blocked[src_i_lo:src_i_hi, src_j_lo:src_j_hi]
            )
    return out


def _snap_to_nearest_free(blocked: np.ndarray,
                          i: int, j: int,
                          search_radius_cells: int = 20,
                          tables_footprint: Optional[np.ndarray] = None,
                          start_ij: Optional[Tuple[int, int]] = None,
                          ) -> Optional[Tuple[int, int]]:
    """If (i, j) is blocked, find the nearest unblocked cell within
    `search_radius_cells`.

    If `tables_footprint` is given, treat this as a GOAL-SIDE snap: the
    result MUST be a cell 8-connected to a cell in `tables_footprint`
    (i.e. sitting on the physical edge of the chosen table).  If no
    free-and-table-adjacent cell exists within the search radius, return
    None — that's the "goal is unreachable / walled off" signal.

    Without `tables_footprint` (start-side snap), any nearby free cell
    will do.

    `start_ij`: when supplied with `tables_footprint`, scores candidate
    cells by their distance to the start cell (so the planner approaches
    the table from the **near side** rather than landing on whichever
    table edge happens to be closest to the table centroid).  Without
    this, the goal-snap could pick a free cell on the FAR side of a
    table, forcing the robot to walk a long loop around and turn 180°
    to face the table from the back — observed live 2026-05-13:
    robot routed 3.25 m around a central obstacle to grasp a bistro
    table only 2.10 m away, ended facing backward, depth camera locked
    onto the wrong surface and the wristless-grasp pipeline reached
    for empty air.
    """
    nx, ny = blocked.shape
    if not (0 <= i < nx and 0 <= j < ny):
        return None
    if not blocked[i, j]:
        # Start-side snap: accept the cell as-is.
        # Goal-side snap: still need to confirm adjacency to a table cell.
        if tables_footprint is None:
            return (i, j)
        if _cell_touches_mask(tables_footprint, i, j):
            return (i, j)
        # Fall through to radial search for a table-adjacent cell.
    near_side_snap = tables_footprint is not None and start_ij is not None
    best: Optional[Tuple[int, int]] = None
    best_score = float("inf")
    for di in range(-search_radius_cells, search_radius_cells + 1):
        for dj in range(-search_radius_cells, search_radius_cells + 1):
            ii, jj = i + di, j + dj
            if not (0 <= ii < nx and 0 <= jj < ny):
                continue
            if blocked[ii, jj]:
                continue
            if tables_footprint is not None:
                if tables_footprint[ii, jj]:
                    # Don't land ON a table cell.
                    continue
                if not _cell_touches_mask(tables_footprint, ii, jj):
                    # Goal-side snap requires table adjacency; skip non-
                    # touching cells entirely (rather than penalising,
                    # which lets walled-off scenes leak through).
                    continue
            if near_side_snap:
                si, sj = start_ij  # type: ignore[misc]
                score = (ii - si) ** 2 + (jj - sj) ** 2
            else:
                score = di * di + dj * dj
            if score < best_score:
                best_score = score
                best = (ii, jj)
    return best


def _cell_touches_mask(mask: np.ndarray, i: int, j: int) -> bool:
    """True if any 8-connected neighbour of (i, j) is True in `mask`."""
    nx, ny = mask.shape
    for dii, djj, _ in _NEIGHBOURS_8:
        ai, aj = i + dii, j + djj
        if 0 <= ai < nx and 0 <= aj < ny and mask[ai, aj]:
            return True
    return False


@dataclass
class PlannedPath:
    waypoints: List[Tuple[float, float]]   # body-frame xy metres
    start_cell: Tuple[int, int]
    goal_cell: Tuple[int, int]
    goal_xy: Tuple[float, float]           # the actual goal coord we planned to


def plan_path(scene: SceneMap,
              start_xy: Tuple[float, float],
              goal_xy: Tuple[float, float],
              clearance_m: float = DEFAULT_CLEARANCE_M,
              unknown_as_obstacle: bool = False,
              unknown_cost_multiplier: float = 3.0,
              start_clearing_radius_m: float = 0.80,
              unknown_traversable_radius_m: Optional[float] = None,
              max_expansions: int = 100_000) -> Optional[PlannedPath]:
    """A* from start to goal on the inflated occupancy grid.

    The goal cell is snapped to the nearest unblocked cell adjacent to a
    TABLE footprint when the raw goal lands on a table cell (which is the
    common case — the goal is the centre of the picked table).

    `unknown_as_obstacle` default was flipped from True to False after live
    testing against the G1's crown-mount Livox MID-360: the LiDAR's
    face-frame occlusion creates a large UNKNOWN donut around the robot
    that, when treated as hard-blocked, pinches every outbound corridor
    shut.  With False, UNKNOWN cells are traversable at
    `unknown_cost_multiplier`×-cost so the planner still prefers routes
    that go through KNOWN-FREE cells when such routes exist.  Set to
    True for aggressive fail-safe behaviour on scenes where the LiDAR
    has clear 360° coverage.

    `start_clearing_radius_m` carves a circular free zone around the
    start cell before A* runs.  The robot itself creates OBSTACLE cells
    within ~0.5 m of the origin (face-frame, chin, torso returns that
    survive the 15 cm SELF_REFLECTION filter); inflating those by the
    robot clearance would otherwise wall the robot in.  The robot is
    standing at `start_xy`, so by construction those cells are passable
    for it.  Default 0.55 m matches the G1 EDU's observed self-reflection
    envelope plus the 0.22 m inflation radius plus a small margin.
    Live-tested: 0.80 m is needed to reliably escape the self-reflection
    ring on this robot; 0.55 m (the earlier pick) left the clearing
    small enough that the inflated self-reflection obstacles still
    blocked every outbound cell.

    `unknown_traversable_radius_m` bounds how far from the start an
    UNKNOWN cell can be traversed.  Set to a finite value (e.g. 1.5 m)
    and UNKNOWN cells farther than that are hard-blocked; UNKNOWN cells
    inside the radius stay traversable at `unknown_cost_multiplier`× cost.
    Fixes the Test-5 failure mode: the face frame's left-side
    occlusion created a large UNKNOWN quadrant, A* routed through it at
    soft cost, and the robot walked 2.3 m into an unscanned area where
    a standing desk was sitting invisibly.  Bounding traversal to the
    near field makes the planner bail with "no path" when the direct
    corridor is blocked and the only open route runs through blind
    space — which is the correct behaviour.  Default `None` preserves
    pre-fix behaviour; live callers should pass 1.5 m or similar.  Has
    no effect when `unknown_as_obstacle=True` (which hard-blocks ALL
    UNKNOWN, not just the far-field ones).
    """
    nx, ny = scene.grid.shape
    blocked = _inflate_obstacles(scene.grid, scene.cell_m, clearance_m,
                                  unknown_as_obstacle)
    tables_fp = scene.grid == TABLE

    start = scene.xy_to_cell(*start_xy)
    goal = scene.xy_to_cell(*goal_xy)
    if start is None or goal is None:
        return None

    # Carve the start clearing: the robot is physically at start_xy, so
    # cells within `start_clearing_radius_m` of it are by definition
    # traversable — anything the LiDAR reads there is the robot's own
    # body (which moves with the robot and does not obstruct it).
    if start_clearing_radius_m > 0.0:
        r_cells = int(math.ceil(start_clearing_radius_m / scene.cell_m))
        si, sj = start
        for di in range(-r_cells, r_cells + 1):
            for dj in range(-r_cells, r_cells + 1):
                if di * di + dj * dj > r_cells * r_cells:
                    continue
                ii, jj = si + di, sj + dj
                if 0 <= ii < nx and 0 <= jj < ny:
                    blocked[ii, jj] = False

    # Near-field UNKNOWN policy.  When
    # `unknown_traversable_radius_m` is set, UNKNOWN cells beyond that
    # radius from the start are treated as hard-blocked — prevents A*
    # from routing through distant LiDAR blind spots.  Near-field
    # UNKNOWN (within the radius) stays traversable at soft cost via
    # the unknown_mask below so the face-frame donut still lets the
    # robot leave the start.
    if (unknown_traversable_radius_m is not None
            and not unknown_as_obstacle):
        si, sj = start
        r_cells_uf = int(math.ceil(
            float(unknown_traversable_radius_m) / scene.cell_m,
        ))
        ii_grid, jj_grid = np.indices((nx, ny))
        dist2 = (ii_grid - si) ** 2 + (jj_grid - sj) ** 2
        far_unknown = (scene.grid == UNKNOWN) & (dist2 > r_cells_uf ** 2)
        blocked = blocked | far_unknown

    # For the soft-cost path: cells that are UNKNOWN (and therefore not
    # hard-blocked) still get a step-cost penalty so the planner prefers
    # KNOWN-FREE routes when both exist.
    unknown_mask = (scene.grid == UNKNOWN) & (~blocked)

    start_adj = _snap_to_nearest_free(blocked, *start,
                                      search_radius_cells=3,
                                      tables_footprint=None)
    # Near-side preference (2026-05-13): pass start_ij so the
    # goal-snap picks the table-adjacent free cell closest to the
    # robot's start pose, not just the one closest to the table
    # centroid.  Without this, A* can route to the far side of a
    # table, the post-walk turn_to swings the robot 180° to face
    # the table from behind, and the depth camera then locks onto
    # whatever non-target surface is in front of THAT pose — the
    # caller reaches for empty air at a phantom table.
    goal_adj = _snap_to_nearest_free(blocked, *goal,
                                     search_radius_cells=20,
                                     tables_footprint=tables_fp,
                                     start_ij=start_adj)
    if start_adj is None or goal_adj is None:
        return None

    def h(a: Tuple[int, int]) -> float:
        return math.hypot(a[0] - goal_adj[0], a[1] - goal_adj[1])

    open_heap: List[Tuple[float, int, Tuple[int, int]]] = []
    counter = 0
    heapq.heappush(open_heap, (h(start_adj), counter, start_adj))
    came_from: dict = {start_adj: None}
    g_cost: dict = {start_adj: 0.0}
    expansions = 0

    while open_heap:
        _, _, cur = heapq.heappop(open_heap)
        if cur == goal_adj:
            break
        expansions += 1
        if expansions > max_expansions:
            return None
        ci, cj = cur
        for di, dj, step_cost in _NEIGHBOURS_8:
            ni, nj = ci + di, cj + dj
            if not (0 <= ni < nx and 0 <= nj < ny):
                continue
            if blocked[ni, nj]:
                continue
            # UNKNOWN cells traversable but penalised, so A* prefers
            # KNOWN-FREE routes where they exist.
            local_cost = step_cost * (
                unknown_cost_multiplier if unknown_mask[ni, nj] else 1.0
            )
            tentative = g_cost[cur] + local_cost
            nxt = (ni, nj)
            if tentative < g_cost.get(nxt, float("inf")):
                g_cost[nxt] = tentative
                came_from[nxt] = cur
                counter += 1
                heapq.heappush(open_heap, (tentative + h(nxt), counter, nxt))
    else:
        return None

    if goal_adj not in came_from:
        return None

    # Reconstruct in cell coordinates.
    path_cells: List[Tuple[int, int]] = []
    node: Optional[Tuple[int, int]] = goal_adj
    while node is not None:
        path_cells.append(node)
        node = came_from.get(node)
    path_cells.reverse()

    # Convert to body-frame xy waypoints.
    waypoints = [scene.cell_to_xy(i, j) for (i, j) in path_cells]
    # Replace the FIRST waypoint with the actual start xy (so the caller
    # starts from where they really are, not the cell centre).
    waypoints[0] = (float(start_xy[0]), float(start_xy[1]))

    # Simplify: drop intermediate waypoints that are collinear with their
    # neighbours (within a small tolerance).  This keeps the output
    # walkable without requiring the caller to hit every 10 cm pixel.
    waypoints = _simplify_collinear(waypoints)

    return PlannedPath(
        waypoints=waypoints,
        start_cell=start_adj,
        goal_cell=goal_adj,
        goal_xy=scene.cell_to_xy(*goal_adj),
    )


def _simplify_collinear(waypoints: List[Tuple[float, float]],
                        tol_rad: float = 0.15) -> List[Tuple[float, float]]:
    """Drop interior waypoints whose direction-change from the previous
    segment is smaller than `tol_rad`.  First and last points are always
    preserved.
    """
    if len(waypoints) <= 2:
        return list(waypoints)
    kept = [waypoints[0]]
    prev_dir: Optional[Tuple[float, float]] = None
    for i in range(1, len(waypoints)):
        cur = waypoints[i]
        prev = kept[-1]
        dx = cur[0] - prev[0]
        dy = cur[1] - prev[1]
        mag = math.hypot(dx, dy)
        if mag < 1e-9:
            continue
        this_dir = (dx / mag, dy / mag)
        if prev_dir is not None and i < len(waypoints) - 1:
            dot = (prev_dir[0] * this_dir[0] + prev_dir[1] * this_dir[1])
            dot = max(-1.0, min(1.0, dot))
            turn = math.acos(dot)
            if turn < tol_rad:
                # same direction — merge by replacing last with cur
                kept[-1] = cur
                prev_dir = this_dir
                continue
        kept.append(cur)
        prev_dir = this_dir
    # Ensure last point kept.
    if kept[-1] != waypoints[-1]:
        kept.append(waypoints[-1])
    return kept


# ── Label name helper (for diagnostics / rendering) ─────────────────────────

_LABEL_NAMES = {FREE: "FREE", OBSTACLE: "OBSTACLE",
                TABLE: "TABLE", UNKNOWN: "UNKNOWN"}


def label_name(v: int) -> str:
    return _LABEL_NAMES.get(int(v), f"?{v}")
