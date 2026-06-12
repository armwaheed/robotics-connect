#!/usr/bin/env python3
"""Offline tests for occupancy-grid A* planning + Navigator. No hardware.

Run: ``python3 test_navigation.py`` (needs numpy).
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from locomotion import SimLocomotion  # noqa: E402
from navigation import Navigator, build_grid, astar  # noqa: E402

EMPTY = np.zeros((0, 3), dtype=float)


def _wall(x, y0, y1, z=0.5, step=0.05):
    """A vertical wall of points at ``x`` spanning ``[y0, y1]`` at height ``z``."""
    ys = np.arange(y0, y1 + step, step)
    return np.column_stack([np.full_like(ys, x), ys, np.full_like(ys, z)])


def test_straight_path_in_empty_space():
    nav = Navigator()
    path = nav.plan(EMPTY, (0.0, 0.0), (2.0, 0.0))
    assert path is not None
    assert math.hypot(path[0][0], path[0][1]) <= 0.1
    assert math.hypot(path[-1][0] - 2.0, path[-1][1]) <= 0.1


def test_routes_around_a_wall():
    # Wall at x=1.0 from y=-1.0 to y=0.3 → robot must detour above y≈0.6.
    nav = Navigator()
    path = nav.plan(_wall(1.0, -1.0, 0.3), (0.0, 0.0), (2.0, 0.0))
    assert path is not None, "no path found around the wall"
    assert max(p[1] for p in path) >= 0.55, "path did not detour around the wall"


def test_goal_inside_obstacle_is_unreachable():
    nav = Navigator()
    assert nav.plan(_wall(1.0, -0.5, 0.5), (0.0, 0.0), (1.0, 0.0)) is None


def test_start_against_obstacle_still_plans():
    # Start cell inflated into the wall; planner must still escape.
    nav = Navigator()
    path = nav.plan(_wall(0.2, -1.0, 1.0), (0.0, 0.0), (-1.0, 0.0))
    assert path is not None and math.hypot(path[-1][0] + 1.0, path[-1][1]) <= 0.1


def test_inflation_blocks_too_narrow_a_gap():
    # 0.3 m gap with a 0.30 m robot radius inflates shut. The wall spans past
    # the bounds so there is no route around its ends → no path at all.
    grid = build_grid(
        np.vstack([_wall(1.0, -3.0, -0.15), _wall(1.0, 0.15, 3.0)]),
        bounds=(-1.0, -2.5, 3.0, 2.5),
    )
    assert astar(grid, (0.0, 0.0), (2.0, 0.0)) is None


def test_navigator_drives_sim_around_obstacle():
    nav = Navigator(goal_tolerance_m=0.2)
    loco = SimLocomotion()
    cloud = _wall(1.0, -1.0, 0.3)
    reached = nav.navigate(loco, (2.0, 0.0), lambda: cloud,
                           vmax=1.0, tolerance_m=0.15, timeout_s=10.0)
    assert reached, "navigator did not reach the goal"
    p = loco.pose()
    assert math.hypot(p.x - 2.0, p.y) <= 0.25


TESTS = [v for k, v in sorted(globals().items()) if k.startswith("test_")]


def main() -> int:
    failures = 0
    for t in TESTS:
        try:
            t()
            print(f"  PASS  {t.__name__}")
        except AssertionError as exc:
            failures += 1
            print(f"  FAIL  {t.__name__}: {exc}")
    print(f"\n{len(TESTS) - failures}/{len(TESTS)} passed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
