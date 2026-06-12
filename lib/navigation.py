"""Navigation — a sensor-agnostic LiDAR-to-path planner for any humanoid.

Pipeline: point cloud → inflated 2D occupancy grid → A* waypoints to the goal →
drive a :class:`~locomotion.LocomotionController` along them, re-planning when a
segment stalls. The *same* numpy runs on a simulator's RayCaster cloud or a real
LiDAR cloud — the planner only needs an ``(N, 3)`` array of points in the robot's
planar frame (``+x`` forward, ``+y`` left, ``+z`` up).

This is the lightweight, dependency-light planner (numpy only). For drift-free
cross-room navigation the localization backend swaps to LiDAR-inertial odometry
(Point-LIO / FAST-LIO) and planning to Nav2 with no change to the consumer API —
see the module's tracking issue.
"""

from __future__ import annotations

import heapq
import math
from collections import deque
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from locomotion import LocomotionController

# ── Tunables ────────────────────────────────────────────────────────────────
GRID_RESOLUTION_M = 0.05         # occupancy cell size
OBSTACLE_Z_BAND = (0.05, 1.80)   # keep points between floor and overhead
ROBOT_RADIUS_M = 0.30            # obstacle inflation (half the robot's width)
GOAL_TOLERANCE_M = 0.15          # arrival radius at the final goal
_SQRT2 = math.sqrt(2.0)
# 8-connected moves as (drow, dcol, step-cost-in-cells).
_NEIGHBOURS = (
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, _SQRT2), (1, -1, _SQRT2), (-1, 1, _SQRT2), (-1, -1, _SQRT2),
)


@dataclass
class OccupancyGrid:
    """A boolean occupancy grid. ``occupied[row, col]`` True ⇒ blocked.

    ``col`` runs with world ``+x``, ``row`` with world ``+y``; cell ``(0, 0)``
    is centred at ``origin``.
    """

    occupied: np.ndarray
    resolution: float
    origin: tuple[float, float]

    @property
    def shape(self) -> tuple[int, int]:
        return self.occupied.shape

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        col = int(round((x - self.origin[0]) / self.resolution))
        row = int(round((y - self.origin[1]) / self.resolution))
        return row, col

    def cell_to_world(self, row: int, col: int) -> tuple[float, float]:
        return (self.origin[0] + col * self.resolution,
                self.origin[1] + row * self.resolution)

    def in_bounds(self, row: int, col: int) -> bool:
        h, w = self.occupied.shape
        return 0 <= row < h and 0 <= col < w

    def is_free(self, row: int, col: int) -> bool:
        return self.in_bounds(row, col) and not self.occupied[row, col]


def build_grid(
    cloud: np.ndarray,
    bounds: tuple[float, float, float, float],
    resolution: float = GRID_RESOLUTION_M,
    z_band: tuple[float, float] = OBSTACLE_Z_BAND,
    inflate_radius_m: float = ROBOT_RADIUS_M,
) -> OccupancyGrid:
    """Project a cloud onto an inflated occupancy grid over ``bounds``.

    ``bounds`` is ``(xmin, ymin, xmax, ymax)`` in world metres. Points outside
    ``z_band`` (floor, overhead) are ignored; obstacles are inflated by
    ``inflate_radius_m`` so the robot can be planned as a point.
    """
    xmin, ymin, xmax, ymax = bounds
    width = max(1, int(math.ceil((xmax - xmin) / resolution)) + 1)
    height = max(1, int(math.ceil((ymax - ymin) / resolution)) + 1)
    occupied = np.zeros((height, width), dtype=bool)

    pts = np.asarray(cloud, dtype=float).reshape(-1, 3)
    if pts.size:
        z = pts[:, 2]
        keep = (z >= z_band[0]) & (z <= z_band[1])
        xy = pts[keep, :2]
        cols = np.round((xy[:, 0] - xmin) / resolution).astype(int)
        rows = np.round((xy[:, 1] - ymin) / resolution).astype(int)
        inside = (rows >= 0) & (rows < height) & (cols >= 0) & (cols < width)
        occupied[rows[inside], cols[inside]] = True

    occupied = _inflate(occupied, int(round(inflate_radius_m / resolution)))
    return OccupancyGrid(occupied, resolution, (xmin, ymin))


def _inflate(occupied: np.ndarray, radius_cells: int) -> np.ndarray:
    """Binary dilation by a disk of ``radius_cells`` (numpy-only, no scipy)."""
    if radius_cells <= 0:
        return occupied
    h, w = occupied.shape
    out = occupied.copy()
    for dr in range(-radius_cells, radius_cells + 1):
        for dc in range(-radius_cells, radius_cells + 1):
            if dr * dr + dc * dc > radius_cells * radius_cells:
                continue
            dr0, dr1 = max(0, dr), h + min(0, dr)
            sr0, sr1 = max(0, -dr), h + min(0, -dr)
            dc0, dc1 = max(0, dc), w + min(0, dc)
            sc0, sc1 = max(0, -dc), w + min(0, -dc)
            out[dr0:dr1, dc0:dc1] |= occupied[sr0:sr1, sc0:sc1]
    return out


def astar(grid: OccupancyGrid, start_xy: tuple[float, float],
          goal_xy: tuple[float, float]) -> list[tuple[float, float]] | None:
    """A* on the 8-connected grid. Returns world waypoints, or None if blocked.

    If the start cell is itself blocked (the robot began inside an obstacle's
    inflation), the search escapes to the nearest free cell first; the goal
    must be free.
    """
    start = grid.world_to_cell(*start_xy)
    goal = grid.world_to_cell(*goal_xy)
    if not grid.in_bounds(*start) or not grid.in_bounds(*goal):
        return None
    if grid.occupied[goal]:
        return None

    prefix: list[tuple[float, float]] = []
    if grid.occupied[start]:
        escaped = _nearest_free(grid, start)
        if escaped is None:
            return None
        prefix = [grid.cell_to_world(*start)]
        start = escaped

    def heuristic(cell: tuple[int, int]) -> float:
        return math.hypot(cell[0] - goal[0], cell[1] - goal[1])

    open_heap: list[tuple[float, tuple[int, int]]] = [(heuristic(start), start)]
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    g_score = {start: 0.0}

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current == goal:
            cells = _trace(came_from, current)
            return _simplify(prefix + [grid.cell_to_world(r, c) for r, c in cells])
        for drow, dcol, step in _NEIGHBOURS:
            nb = (current[0] + drow, current[1] + dcol)
            if not grid.is_free(*nb):
                continue
            tentative = g_score[current] + step
            if tentative < g_score.get(nb, math.inf):
                came_from[nb] = current
                g_score[nb] = tentative
                heapq.heappush(open_heap, (tentative + heuristic(nb), nb))
    return None


def _trace(came_from: dict, current: tuple[int, int]) -> list[tuple[int, int]]:
    cells = [current]
    while current in came_from:
        current = came_from[current]
        cells.append(current)
    cells.reverse()
    return cells


def _nearest_free(grid: OccupancyGrid, cell: tuple[int, int]
                  ) -> tuple[int, int] | None:
    """Breadth-first search for the closest free cell to ``cell``."""
    seen = {cell}
    queue = deque([cell])
    while queue:
        current = queue.popleft()
        for drow, dcol, _ in _NEIGHBOURS:
            nb = (current[0] + drow, current[1] + dcol)
            if nb in seen or not grid.in_bounds(*nb):
                continue
            seen.add(nb)
            if not grid.occupied[nb]:
                return nb
            queue.append(nb)
    return None


def _simplify(path: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Drop interior waypoints that lie on the same heading (collinear merge)."""
    if len(path) <= 2:
        return path
    out = [path[0]]
    for prev, cur, nxt in zip(path, path[1:], path[2:]):
        before = math.atan2(cur[1] - prev[1], cur[0] - prev[0])
        after = math.atan2(nxt[1] - cur[1], nxt[0] - cur[0])
        if abs((after - before + math.pi) % (2.0 * math.pi) - math.pi) > 1e-3:
            out.append(cur)
    out.append(path[-1])
    return out


class Navigator:
    """Plan a path from a cloud and drive a controller along it."""

    def __init__(
        self,
        resolution: float = GRID_RESOLUTION_M,
        z_band: tuple[float, float] = OBSTACLE_Z_BAND,
        robot_radius_m: float = ROBOT_RADIUS_M,
        goal_tolerance_m: float = GOAL_TOLERANCE_M,
    ) -> None:
        self.resolution = resolution
        self.z_band = z_band
        self.robot_radius_m = robot_radius_m
        self.goal_tolerance_m = goal_tolerance_m

    def plan(self, cloud: np.ndarray, start_xy: tuple[float, float],
             goal_xy: tuple[float, float]) -> list[tuple[float, float]] | None:
        """Plan world-frame waypoints from ``start_xy`` to ``goal_xy``."""
        grid = build_grid(cloud, self._bounds(cloud, start_xy, goal_xy),
                          self.resolution, self.z_band, self.robot_radius_m)
        return astar(grid, start_xy, goal_xy)

    def navigate(self, loco: LocomotionController,
                 goal_xy: tuple[float, float], cloud_source,
                 max_replans: int = 5, **walk_kwargs) -> bool:
        """Drive ``loco`` to ``goal_xy``, re-planning on a stalled segment.

        ``cloud_source`` is a zero-arg callable returning the latest ``(N, 3)``
        cloud. Returns True on arrival, False if no plan exists or a segment
        times out / the re-plan budget is exhausted.
        """
        for _ in range(max_replans + 1):
            p = loco.pose()
            if math.hypot(goal_xy[0] - p.x, goal_xy[1] - p.y) <= self.goal_tolerance_m:
                return True
            waypoints = self.plan(cloud_source(), (p.x, p.y), goal_xy)
            if not waypoints:
                return False
            stalled = False
            for wp in waypoints[1:]:
                here = loco.pose()
                loco.turn_to(math.atan2(wp[1] - here.y, wp[0] - here.x))
                result = loco.walk_to(wp, **walk_kwargs)
                if result == "timeout":
                    return False
                if result is not None:  # stall reason → re-plan from here
                    stalled = True
                    break
            if not stalled:
                p = loco.pose()
                return math.hypot(goal_xy[0] - p.x, goal_xy[1] - p.y) <= self.goal_tolerance_m
        return False

    def _bounds(self, cloud: np.ndarray, start_xy, goal_xy
                ) -> tuple[float, float, float, float]:
        pad = self.robot_radius_m + 0.5
        xs = [start_xy[0], goal_xy[0]]
        ys = [start_xy[1], goal_xy[1]]
        pts = np.asarray(cloud, dtype=float).reshape(-1, 3)
        if pts.size:
            z = pts[:, 2]
            xy = pts[(z >= self.z_band[0]) & (z <= self.z_band[1]), :2]
            if xy.size:
                xs += [float(xy[:, 0].min()), float(xy[:, 0].max())]
                ys += [float(xy[:, 1].min()), float(xy[:, 1].max())]
        return min(xs) - pad, min(ys) - pad, max(xs) + pad, max(ys) + pad
