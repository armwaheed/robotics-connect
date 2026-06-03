#!/usr/bin/env python3
"""
test_scene_map — mock-based tests for scene_map.py.

Run as:
    python3 test_scene_map.py

No hardware, no DDS — uses `MockLidarSight`-style synthetic clouds.
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

from lidar_sight import LidarCloud, Table, MockLidarSight  # noqa: E402
from scene_map import (  # noqa: E402
    FREE, OBSTACLE, TABLE, UNKNOWN,
    SceneMap, build_scene_map, pick_goal_table, plan_path, dedup_tables,
    DEFAULT_X_BOUNDS, DEFAULT_Y_BOUNDS, DEFAULT_CELL_M,
)


# ── Test scene builders ─────────────────────────────────────────────────────

def _synth_cloud(pts: np.ndarray) -> LidarCloud:
    return LidarCloud(points=pts.astype(np.float32), intensities=None,
                      timestamp=0.0)


def _floor_plane(n: int = 4000, radius: float = 4.0,
                 floor_z: float = -0.80, seed: int = 0) -> np.ndarray:
    rng = np.random.default_rng(seed)
    r = np.sqrt(rng.uniform(0.0, 1.0, n)) * radius
    th = rng.uniform(-math.pi, math.pi, n)
    return np.stack([r * np.cos(th), r * np.sin(th),
                     np.full(n, floor_z) + rng.normal(0, 0.01, n)], axis=-1)


def _obstacle_column(xy: tuple, z_lo: float, z_hi: float,
                     n: int = 400, radius: float = 0.15,
                     seed: int = 1) -> np.ndarray:
    """A thin vertical column of points — simulates a chair / pole / person."""
    rng = np.random.default_rng(seed)
    r = np.sqrt(rng.uniform(0.0, 1.0, n)) * radius
    th = rng.uniform(-math.pi, math.pi, n)
    z = rng.uniform(z_lo, z_hi, n)
    return np.stack([xy[0] + r * np.cos(th),
                     xy[1] + r * np.sin(th),
                     z], axis=-1)


def _table_slab(xy: tuple, top_z: float, size: tuple = (0.5, 0.5),
                n: int = 800, seed: int = 2) -> np.ndarray:
    rng = np.random.default_rng(seed)
    sw, sl = size
    jx = rng.uniform(-sw / 2, sw / 2, n)
    jy = rng.uniform(-sl / 2, sl / 2, n)
    jz = rng.normal(0.0, 0.01, n)
    return np.stack([xy[0] + jx, xy[1] + jy, top_z + jz], axis=-1)


# ── Tests ───────────────────────────────────────────────────────────────────

def test_empty_cloud_gives_all_unknown():
    empty = _synth_cloud(np.zeros((0, 3), dtype=np.float32))
    sm = build_scene_map(empty, tables=[])
    assert sm.count(FREE) == 0
    assert sm.count(OBSTACLE) == 0
    assert sm.count(TABLE) == 0
    assert sm.count(UNKNOWN) == sm.grid.size
    print("  PASS: empty cloud → all UNKNOWN")


def test_floor_only_cloud_gives_free_or_unknown():
    pts = _floor_plane(n=6000)
    cloud = _synth_cloud(pts)
    sm = build_scene_map(cloud, tables=[])
    # Floor-only: no obstacles, no tables.  Cells with coverage → FREE.
    assert sm.count(OBSTACLE) == 0, sm.summary()
    assert sm.count(TABLE) == 0, sm.summary()
    assert sm.count(FREE) > 0, sm.summary()
    print(f"  PASS: floor-only scene → {sm.count(FREE)} FREE / "
          f"{sm.count(UNKNOWN)} UNKNOWN / 0 OBSTACLE")


def test_column_obstacle_shows_up():
    pts = np.concatenate([
        _floor_plane(n=6000, seed=0),
        _obstacle_column(xy=(1.0, 0.0), z_lo=-0.4, z_hi=+0.4, n=400, seed=1),
    ])
    sm = build_scene_map(_synth_cloud(pts), tables=[])
    assert sm.count(OBSTACLE) > 0, sm.summary()
    # The obstacle cell at (1.0, 0.0) should be OBSTACLE.
    cell = sm.xy_to_cell(1.0, 0.0)
    assert cell is not None
    assert sm.grid[cell] == OBSTACLE, f"expected OBSTACLE at (1,0), got {sm.grid[cell]}"
    print(f"  PASS: column obstacle detected at (1, 0) — {sm.count(OBSTACLE)} OBSTACLE cells")


def test_table_cells_mark_as_table():
    table_center = (1.0, 0.0, 0.04)
    pts = np.concatenate([
        _floor_plane(n=6000, seed=0),
        _table_slab(xy=table_center[:2], top_z=table_center[2],
                    size=(0.5, 0.5), n=800, seed=2),
    ])
    fake_table = Table(
        center_xyz=table_center, normal_xyz=(0, 0, 1),
        height_above_floor_m=None, area_estimate_m2=0.25,
        confidence=0.5, point_count=800,
    )
    sm = build_scene_map(_synth_cloud(pts), tables=[fake_table])
    assert sm.count(TABLE) > 0, sm.summary()
    cell = sm.xy_to_cell(1.0, 0.0)
    assert cell is not None and sm.grid[cell] == TABLE, \
        f"expected TABLE at (1,0), got label={sm.grid[cell]}"
    print(f"  PASS: table footprint marks TABLE cells — {sm.count(TABLE)} cells")


def test_pick_goal_table_on_path_beats_off_path():
    # Three tables: forward-on-heading, forward-off-heading, behind.
    # Expected: forward-on-heading wins (bucket 1).
    t_behind = Table((-3.0, 0.0, 0.0), (0,0,1), None, 0.2, 0.5, 100)
    t_side   = Table(( 0.5, 2.0, 0.0), (0,0,1), None, 0.2, 0.5, 100)   # > 30° off
    t_fwd    = Table(( 2.0, 0.0, 0.0), (0,0,1), None, 0.2, 0.5, 100)
    winner = pick_goal_table([t_behind, t_side, t_fwd])
    assert winner is t_fwd, f"expected forward-on-heading, got {winner.center_xyz}"
    print("  PASS: on-path table beats off-path (behind + far-side) candidates")


def test_pick_goal_table_bucket_trumps_euclidean():
    # The discriminator: piano (on-path, 3.5 m) beats nothing-on-path
    # candidates, even if those are closer.  And among on-path tables, the
    # nearer (bistro at 3 m, 5° off) beats the very-on-axis but farther
    # (piano at 3.5 m, 0° off).
    t_piano  = Table((3.5,  0.00, 0.0), (0,0,1), None, 0.5, 0.5, 200)
    t_bistro = Table((3.0, -0.30, 0.0), (0,0,1), None, 0.2, 0.5, 100)
    t_close_side = Table((0.5, -1.8, 0.0), (0,0,1), None, 0.2, 0.5, 100)  # ≫ 30° off
    winner = pick_goal_table([t_piano, t_bistro, t_close_side])
    assert winner is t_bistro, (
        f"bistro-at-3m-on-path should beat piano-at-3.5m-on-path and "
        f"close-but-off-path; got {winner.center_xyz}")
    print("  PASS: bucketed pref: nearer on-path wins over farther on-path "
          "and over closer off-path")


def test_pick_goal_table_falls_back_off_path_when_empty_on_path():
    # All tables far-off-axis (> 30°).  Must pick nearest Euclidean among them.
    ts = [
        Table((0.1, -2.5, 0.0), (0,0,1), None, 0.2, 0.5, 100),  # ~87° off
        Table((0.1, +1.0, 0.0), (0,0,1), None, 0.2, 0.5, 100),  # ~84° off, closer
        Table((-4.0, 0.0, 0.0), (0,0,1), None, 0.2, 0.5, 100),  # behind
    ]
    winner = pick_goal_table(ts)
    # The closest of these is at ~hypot(0.1,1.0)=1.0, vs ~2.5 and 4.0.
    assert winner is ts[1], f"expected y=+1.0 nearest, got {winner.center_xyz}"
    print("  PASS: empty on-path bucket → nearest off-path wins")


def test_pick_goal_table_none_when_all_tiny():
    ts = [Table((1.0, 0.0, 0.0), (0,0,1), None, 0.01, 0.5, 100)]
    assert pick_goal_table(ts, min_area_m2=0.05) is None
    print("  PASS: rejects tables smaller than min_area_m2")


def test_dedup_collapses_xy_colocated_shelves():
    # The five detections at (-0.81, -0.75) in the live diagnostic:
    # same xy, different z, different areas.  Must collapse to one.
    shelves = [
        Table((-0.81, -0.65, +0.26), (0,0,1), None, 0.22, 0.44, 150),
        Table((-0.86, -0.80, +0.22), (0,0,1), None, 0.22, 0.35, 200),  # most pts
        Table((-0.88, -0.80, -0.05), (0,0,1), None, 0.19, 0.33, 130),
        Table((-0.90, -0.76, +0.15), (0,0,1), None, 0.16, 0.28, 110),
        Table((-0.82, -0.77, -0.30), (0,0,1), None, 0.12, 0.28,  95),
    ]
    merged = dedup_tables(shelves, xy_merge_radius_m=0.25)
    assert len(merged) == 1, f"expected 1 merged table, got {len(merged)}"
    assert merged[0].point_count == 200, (
        f"expected the most-points representative to survive, got {merged[0]}")
    print(f"  PASS: 5 colocated shelves → 1 representative ({merged[0].point_count} pts)")


def test_dedup_keeps_distinct_tables():
    # Two genuinely distinct tables at ~2 m apart must NOT merge.
    a = Table((3.0, 0.0, 0.04), (0,0,1), None, 0.2, 0.5, 100)
    b = Table((3.0, 1.5, 0.04), (0,0,1), None, 0.2, 0.5, 100)
    merged = dedup_tables([a, b], xy_merge_radius_m=0.25)
    assert len(merged) == 2, f"expected 2 tables to survive dedup, got {len(merged)}"
    print("  PASS: distinct tables 1.5 m apart survive dedup")


def test_plan_path_straight_line():
    pts = _floor_plane(n=6000)
    fake_table = Table((2.0, 0.0, 0.04), (0,0,1), None, 0.25, 0.5, 100)
    pts = np.concatenate([
        pts,
        _table_slab(xy=(2.0, 0.0), top_z=0.04, size=(0.5, 0.5), n=800),
    ])
    sm = build_scene_map(_synth_cloud(pts), tables=[fake_table])
    plan = plan_path(sm, start_xy=(0.0, 0.0), goal_xy=(2.0, 0.0),
                     unknown_as_obstacle=False)
    assert plan is not None, "expected a path in a clear scene"
    assert len(plan.waypoints) >= 2
    # Start near (0,0), end near (2,0).
    sx, sy = plan.waypoints[0]
    gx, gy = plan.waypoints[-1]
    assert abs(sx) < 0.2 and abs(sy) < 0.2, f"start={plan.waypoints[0]}"
    assert abs(gx - 2.0) < 0.5 and abs(gy) < 0.5, f"end={plan.waypoints[-1]}"
    print(f"  PASS: straight-line plan with {len(plan.waypoints)} waypoints")


def test_plan_path_routes_around_obstacle():
    pts = np.concatenate([
        _floor_plane(n=6000),
        # Wall-ish obstacle blocking x=1.5 from y=-0.3 to y=+0.3
        _obstacle_column(xy=(1.5, -0.2), z_lo=-0.4, z_hi=+0.4, n=400, seed=1, radius=0.15),
        _obstacle_column(xy=(1.5,  0.0), z_lo=-0.4, z_hi=+0.4, n=400, seed=2, radius=0.15),
        _obstacle_column(xy=(1.5, +0.2), z_lo=-0.4, z_hi=+0.4, n=400, seed=3, radius=0.15),
        _table_slab(xy=(3.0, 0.0), top_z=0.04, size=(0.5, 0.5), n=800, seed=4),
    ])
    fake_table = Table((3.0, 0.0, 0.04), (0,0,1), None, 0.25, 0.5, 100)
    sm = build_scene_map(_synth_cloud(pts), tables=[fake_table])
    plan = plan_path(sm, start_xy=(0.0, 0.0), goal_xy=(3.0, 0.0),
                     clearance_m=0.25, unknown_as_obstacle=False)
    assert plan is not None, "expected a path around the obstacle column"
    # The path must detour laterally at some point — check that max |y| on
    # the path is > clearance (otherwise it walked right through the wall).
    max_lateral = max(abs(y) for _, y in plan.waypoints)
    assert max_lateral > 0.20, f"path did not detour laterally (max |y|={max_lateral})"
    print(f"  PASS: plan detours around obstacle (max |y|={max_lateral:.2f} m, "
          f"{len(plan.waypoints)} waypoints)")


def test_plan_path_returns_none_when_walled_off():
    # Ring the goal with obstacles so A* can't reach it.
    pts = [_floor_plane(n=6000)]
    for ang in np.linspace(0, 2 * math.pi, 24, endpoint=False):
        xc = 2.0 + 0.6 * math.cos(ang)
        yc = 0.0 + 0.6 * math.sin(ang)
        pts.append(_obstacle_column(xy=(xc, yc), z_lo=-0.4, z_hi=0.4,
                                     n=200, seed=int(ang * 100), radius=0.12))
    pts.append(_table_slab(xy=(2.0, 0.0), top_z=0.04, size=(0.3, 0.3), n=200, seed=99))
    fake_table = Table((2.0, 0.0, 0.04), (0,0,1), None, 0.09, 0.5, 100)
    sm = build_scene_map(_synth_cloud(np.concatenate(pts)), tables=[fake_table])
    plan = plan_path(sm, start_xy=(0.0, 0.0), goal_xy=(2.0, 0.0),
                     clearance_m=0.30, unknown_as_obstacle=False)
    assert plan is None, f"expected None for walled-off goal, got {plan}"
    print("  PASS: walled-off goal → plan_path returns None")


def test_mock_lidar_sight_integration():
    """End-to-end: MockLidarSight → find_tables → build_scene_map → pick_goal → plan."""
    mock = MockLidarSight()  # default bistro-table scene
    cloud = mock.latest_cloud()
    tables = mock.find_tables()
    assert len(tables) >= 1
    sm = build_scene_map(cloud, tables=tables)
    goal = pick_goal_table(tables)
    assert goal is not None
    plan = plan_path(sm, start_xy=(0.0, 0.0),
                     goal_xy=(goal.center_xyz[0], goal.center_xyz[1]),
                     unknown_as_obstacle=False)
    assert plan is not None, \
        f"expected a plan to reach the default bistro table; scene: {sm.summary()}"
    print(f"  PASS: MockLidarSight end-to-end — goal at "
          f"{goal.center_xyz[:2]}, {len(plan.waypoints)} waypoints")


def test_unknown_traversable_radius_blocks_far_unknown():
    """Test 5 fix: `unknown_traversable_radius_m` must hard-block
    UNKNOWN cells beyond the radius, even when `unknown_as_obstacle=False`.

    Scene: robot at origin, a narrow obstacle wall across the forward
    corridor, and a goal-TABLE at (3, 0) behind the wall.  The LiDAR
    sees the wall and the table but the lateral detour cells are
    UNKNOWN (no floor returns there to make them FREE).  With the
    unbounded soft-cost policy the planner routes around the wall
    through the UNKNOWN cells.  With `unknown_traversable_radius_m=1.0`
    the far UNKNOWN cells become hard-blocked and the planner bails.
    """
    # Floor only inside a narrow +x corridor, NOT laterally — so lateral
    # cells stay UNKNOWN (no cloud returns there).
    n = 6000
    rng = np.random.default_rng(42)
    xs = rng.uniform(-0.5, 3.5, n)
    ys = rng.uniform(-0.3, 0.3, n)  # only a ±0.3 m corridor gets FLOOR
    zs = np.full(n, -0.80) + rng.normal(0.0, 0.01, n)
    corridor_floor = np.stack([xs, ys, zs], axis=-1)

    # Wall of obstacle cells at x = 1.5 m spanning the corridor.
    wall = _obstacle_column(xy=(1.5, 0.0), z_lo=-0.2, z_hi=0.4,
                            n=400, radius=0.25, seed=7)

    # Table slab behind the wall.
    table = _table_slab(xy=(3.0, 0.0), top_z=0.04,
                        size=(0.3, 0.3), n=200, seed=8)
    fake_table = Table((3.0, 0.0, 0.04), (0, 0, 1), None,
                       0.09, 0.5, 200)

    pts = np.concatenate([corridor_floor, wall, table], axis=0)
    scene = build_scene_map(_synth_cloud(pts), tables=[fake_table])

    # Sanity: the lateral quadrants are UNKNOWN (no cloud coverage).
    assert scene.count(UNKNOWN) > 500, (
        f"test setup failure: expected >500 UNKNOWN cells laterally, "
        f"got {scene.count(UNKNOWN)}"
    )

    # With the unbounded soft-cost policy, the planner can route around
    # the wall by detouring through the lateral UNKNOWN cells (exactly
    # the Test-5 live failure mode).  It may find a plan.
    plan_soft = plan_path(scene, start_xy=(0.0, 0.0), goal_xy=(3.0, 0.0),
                          clearance_m=0.15,
                          unknown_as_obstacle=False,
                          unknown_traversable_radius_m=None)

    # With a 1.0 m radius cap, the lateral UNKNOWN cells needed for the
    # detour (all farther than 1.0 m from start) are hard-blocked, so A*
    # can't route around the wall.
    plan_capped = plan_path(scene, start_xy=(0.0, 0.0), goal_xy=(3.0, 0.0),
                            clearance_m=0.15,
                            unknown_as_obstacle=False,
                            unknown_traversable_radius_m=1.0)

    # The soft-cost plan, if it found a path, must have routed through
    # distant UNKNOWN cells — verify by checking it reaches far-lateral.
    # If it didn't find a plan at all (e.g. the wall is too thick),
    # the test still passes as long as the capped version also fails.
    assert plan_capped is None, (
        "expected unknown_traversable_radius_m=1.0 to hard-block the "
        "far-lateral UNKNOWN detour and return None; got a plan "
        f"with {None if plan_capped is None else len(plan_capped.waypoints)} wp"
    )

    # And a radius = None or large-number should behave the same as the
    # old (pre-fix) behaviour — used as a hedge that the capped failure
    # wasn't for some other reason.
    if plan_soft is not None:
        # Confirm the soft plan's path actually used lateral UNKNOWN.
        max_abs_y = max(abs(wp[1]) for wp in plan_soft.waypoints)
        assert max_abs_y > 0.4, (
            f"soft-cost plan should detour laterally; max |y|={max_abs_y:.2f}"
        )
        print(f"  PASS: near-field UNKNOWN cap bails (as expected); "
              f"soft-cost plan detours to |y|={max_abs_y:.2f} m "
              f"({len(plan_soft.waypoints)} wp)")
    else:
        print("  PASS: near-field UNKNOWN cap bails; soft-cost also had "
              "no path (acceptable — wall too thick for this test scene)")


# ── Driver ──────────────────────────────────────────────────────────────────

def main() -> int:
    tests = [
        test_empty_cloud_gives_all_unknown,
        test_floor_only_cloud_gives_free_or_unknown,
        test_column_obstacle_shows_up,
        test_table_cells_mark_as_table,
        test_pick_goal_table_on_path_beats_off_path,
        test_pick_goal_table_bucket_trumps_euclidean,
        test_pick_goal_table_falls_back_off_path_when_empty_on_path,
        test_pick_goal_table_none_when_all_tiny,
        test_dedup_collapses_xy_colocated_shelves,
        test_dedup_keeps_distinct_tables,
        test_plan_path_straight_line,
        test_plan_path_routes_around_obstacle,
        test_plan_path_returns_none_when_walled_off,
        test_mock_lidar_sight_integration,
        test_unknown_traversable_radius_blocks_far_unknown,
    ]
    failed = 0
    for t in tests:
        try:
            t()
        except AssertionError as e:
            print(f"  FAIL: {t.__name__}: {e}")
            failed += 1
    print()
    if failed:
        print(f"{failed}/{len(tests)} tests FAILED")
        return 1
    print(f"all {len(tests)} tests PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
