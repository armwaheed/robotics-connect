#!/usr/bin/env python3
"""
_diag_plan — live diagnostic for navigation perception + planning.

Captures N frames from the live LiDAR (or MockLidarSight), runs
`find_tables → pick_goal_table → build_scene_map → plan_path`, and
renders a 4-panel JPEG per frame:

    top-left   Top-down point cloud (body +X fwd, +Y left), z colour-coded,
               table centroids and goal overlaid
    top-right  Occupancy grid with FREE / OBSTACLE / TABLE / UNKNOWN
               colours, inflated-blocked mask dashed, planned path drawn,
               start + goal markers
    bot-left   Side view (x, z) showing floor, navigable band, tables
    bot-right  Legend + text summary (cloud size, goal choice, plan length,
               path arc-length, SceneMap state counts)

Run on the robot:

    conda activate unitree_deploy
    export CYCLONEDDS_URI=file:///home/unitree/cyclonedds.xml
    cd /home/unitree/robotics-connect/lidar_sight
    python3 _diag_plan.py --n 3 --accumulate 5 --out /tmp/plan_frames

Or off-robot against the mock:

    python3 _diag_plan.py --mock --out /tmp/plan_frames
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from typing import List, Optional

import numpy as np

import matplotlib
matplotlib.use("Agg")  # headless
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Rectangle

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

from lidar_sight import LidarSight, MockLidarSight, Table  # noqa: E402
from scene_map import (  # noqa: E402
    FREE, OBSTACLE, TABLE, UNKNOWN,
    SceneMap, build_scene_map, pick_goal_table, plan_path,
    DEFAULT_CLEARANCE_M, DEFAULT_CELL_M,
    _inflate_obstacles,  # internal helper we reuse for the rendered mask
)


# ── Colour maps ─────────────────────────────────────────────────────────────

# FREE → light grey, OBSTACLE → red, TABLE → green, UNKNOWN → dark grey
_LABEL_CMAP = ListedColormap(["#e8e8e8", "#d62728", "#2ca02c", "#606060"])


# ── Rendering ──────────────────────────────────────────────────────────────

def _render_frame(out_path: str,
                  cloud_pts: np.ndarray,
                  tables: List[Table],
                  goal: Optional[Table],
                  scene: SceneMap,
                  inflated: np.ndarray,
                  plan,
                  frame_idx: int,
                  timestamp: float):
    fig = plt.figure(figsize=(14, 10), dpi=120)
    fig.suptitle(
        f"Navigation plan — frame {frame_idx:03d}  "
        f"{cloud_pts.shape[0]} pts  t={timestamp:.2f}",
        fontsize=11,
    )

    # ── Top-down point cloud + tables + goal ────────────────────────────
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.set_title("Top-down  +X fwd  +Y left", fontsize=10)
    # Filter to the SceneMap bounds so we plot only what we planned over.
    xl, xh = scene.x_bounds
    yl, yh = scene.y_bounds
    mask = ((cloud_pts[:, 0] >= xl) & (cloud_pts[:, 0] <= xh)
            & (cloud_pts[:, 1] >= yl) & (cloud_pts[:, 1] <= yh))
    p = cloud_pts[mask]
    sc = ax1.scatter(p[:, 0], p[:, 1], c=p[:, 2], s=1.0, cmap="turbo",
                     vmin=scene.floor_z, vmax=scene.nav_z_max, alpha=0.5)
    # Robot marker at origin
    ax1.plot(0, 0, "ko", markersize=6)
    ax1.annotate("robot", (0, 0), textcoords="offset points", xytext=(6, 6))
    # Tables
    for t in tables:
        is_goal = (goal is not None and t is goal)
        colour = "lime" if is_goal else "gold"
        ax1.plot(t.center_xyz[0], t.center_xyz[1], "P", markersize=10,
                 markerfacecolor=colour, markeredgecolor="black")
        ax1.annotate(
            f"table{' (GOAL)' if is_goal else ''}\n"
            f"z={t.center_xyz[2]:+.2f} a={t.area_estimate_m2:.2f}m²",
            (t.center_xyz[0], t.center_xyz[1]),
            textcoords="offset points", xytext=(8, -2), fontsize=7,
        )
    # Path
    if plan is not None:
        xs = [w[0] for w in plan.waypoints]
        ys = [w[1] for w in plan.waypoints]
        ax1.plot(xs, ys, "b-", linewidth=1.8, label="planned path")
        ax1.plot(xs, ys, "b.", markersize=4)
        ax1.plot(xs[-1], ys[-1], "b*", markersize=14,
                 markerfacecolor="cyan", markeredgecolor="black",
                 label="end of path")
    ax1.set_xlabel("x [m]")
    ax1.set_ylabel("y [m]")
    ax1.set_xlim(xl, xh)
    ax1.set_ylim(yl, yh)
    ax1.set_aspect("equal", adjustable="box")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="lower right", fontsize=7)
    plt.colorbar(sc, ax=ax1, shrink=0.7, label="body z [m]")

    # ── Occupancy grid ──────────────────────────────────────────────────
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.set_title("Occupancy grid  (blocked = inflated obstacle)", fontsize=10)
    # Transpose so x is horizontal, y is vertical.  Pcolormesh uses edges.
    xs = np.linspace(xl, xh, scene.grid.shape[0] + 1)
    ys = np.linspace(yl, yh, scene.grid.shape[1] + 1)
    # Show the label grid.
    ax2.pcolormesh(xs, ys, scene.grid.T, cmap=_LABEL_CMAP, vmin=0, vmax=3,
                   shading="flat")
    # Overlay the inflated-blocked mask as a translucent dark wash.  The
    # old formulation added one `Rectangle` patch per newly-blocked cell
    # in a Python loop — a 60x60 grid with dense obstacles can produce
    # ~500-1000 rectangles, taking ~5 s per render on the Jetson and
    # dominating the pre-walk wait time (2026-04-17 bench).
    # An RGBA imshow overlay gets the same "this cell is inflation-
    # blocked" visual signal in one vectorised call (~100 ms on the
    # same grid).
    newly_blocked = inflated & (scene.grid != OBSTACLE)
    if newly_blocked.any():
        # Shape: grid is (nx, ny), displayed with x horizontal, y
        # vertical → transpose before imshow so axes line up.
        nx_b, ny_b = newly_blocked.shape
        overlay = np.zeros((ny_b, nx_b, 4), dtype=np.float32)
        alpha = np.where(newly_blocked.T, 0.35, 0.0).astype(np.float32)
        # RGB = (0, 0, 0) → dark mask; A from the newly_blocked mask.
        overlay[..., 3] = alpha
        ax2.imshow(overlay, extent=(xl, xh, yl, yh), origin="lower",
                   interpolation="nearest", zorder=2)
    # Robot marker + goal star + path.
    ax2.plot(0, 0, "ko", markersize=6)
    if goal is not None:
        ax2.plot(goal.center_xyz[0], goal.center_xyz[1], "P", markersize=10,
                 markerfacecolor="lime", markeredgecolor="black")
    if plan is not None:
        xs_p = [w[0] for w in plan.waypoints]
        ys_p = [w[1] for w in plan.waypoints]
        ax2.plot(xs_p, ys_p, "b-", linewidth=2.0)
        ax2.plot(xs_p, ys_p, "b.", markersize=5)
        ax2.plot(xs_p[-1], ys_p[-1], "b*", markersize=14,
                 markerfacecolor="cyan", markeredgecolor="black")
    ax2.set_xlabel("x [m]")
    ax2.set_ylabel("y [m]")
    ax2.set_xlim(xl, xh)
    ax2.set_ylim(yl, yh)
    ax2.set_aspect("equal", adjustable="box")
    ax2.grid(True, alpha=0.2, linewidth=0.3)
    # Legend for cell labels
    from matplotlib.patches import Patch
    legend_elems = [
        Patch(facecolor="#e8e8e8", edgecolor="black", label="FREE"),
        Patch(facecolor="#d62728", edgecolor="black", label="OBSTACLE"),
        Patch(facecolor="#2ca02c", edgecolor="black", label="TABLE"),
        Patch(facecolor="#606060", edgecolor="black", label="UNKNOWN"),
        Patch(facecolor="white", edgecolor="black", linestyle="--",
              label="inflated (blocked)"),
    ]
    ax2.legend(handles=legend_elems, loc="lower right", fontsize=7)

    # ── Side view (XZ), + floor + navigable band ────────────────────────
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.set_title("Side (XZ)  +X fwd  +Z up", fontsize=10)
    ax3.scatter(p[:, 0], p[:, 2], c=p[:, 1], s=0.8, cmap="coolwarm",
                vmin=yl, vmax=yh, alpha=0.5)
    # Floor line
    ax3.axhline(scene.floor_z, color="green", linestyle="--", linewidth=1,
                label=f"floor z={scene.floor_z:+.3f}")
    ax3.axhspan(scene.nav_z_min, scene.nav_z_max, color="red", alpha=0.05,
                label=f"navigable band [{scene.nav_z_min:+.2f}, "
                      f"{scene.nav_z_max:+.2f}]")
    # Tables
    for t in tables:
        is_goal = (goal is not None and t is goal)
        ax3.plot(t.center_xyz[0], t.center_xyz[2], "P", markersize=9,
                 markerfacecolor="lime" if is_goal else "gold",
                 markeredgecolor="black")
    ax3.set_xlabel("x [m]")
    ax3.set_ylabel("z [m]")
    ax3.set_xlim(xl, xh)
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc="lower right", fontsize=7)

    # ── Summary text ────────────────────────────────────────────────────
    ax4 = fig.add_subplot(2, 2, 4)
    ax4.axis("off")
    path_len = 0.0
    if plan is not None:
        for i in range(1, len(plan.waypoints)):
            dx = plan.waypoints[i][0] - plan.waypoints[i - 1][0]
            dy = plan.waypoints[i][1] - plan.waypoints[i - 1][1]
            path_len += (dx * dx + dy * dy) ** 0.5
    lines = [
        "─" * 64,
        f"  Frame {frame_idx:03d}    t={timestamp:.3f}",
        f"  Cloud: {cloud_pts.shape[0]} body-frame points",
        "",
        f"  SceneMap: cells={scene.grid.shape}  cell_m={scene.cell_m:.2f}",
        f"    FREE={scene.count(FREE)}   OBSTACLE={scene.count(OBSTACLE)}",
        f"    TABLE={scene.count(TABLE)}   UNKNOWN={scene.count(UNKNOWN)}",
        f"    floor_z={scene.floor_z:+.3f}  "
        f"nav_z=[{scene.nav_z_min:+.3f}, {scene.nav_z_max:+.3f}]",
        "",
        f"  Tables: {len(tables)} found",
    ]
    for i, t in enumerate(tables[:5]):
        tag = " ← GOAL" if (goal is not None and t is goal) else ""
        lines.append(
            f"    #{i}: ({t.center_xyz[0]:+.2f}, {t.center_xyz[1]:+.2f}, "
            f"{t.center_xyz[2]:+.2f})  area={t.area_estimate_m2:.2f}m²"
            f"  conf={t.confidence:.2f}{tag}"
        )
    if len(tables) > 5:
        lines.append(f"    ... {len(tables) - 5} more")
    lines.append("")
    if goal is None:
        lines.append("  Goal:      (no eligible table)")
    else:
        lines.append(f"  Goal:      ({goal.center_xyz[0]:+.2f}, "
                     f"{goal.center_xyz[1]:+.2f})  area={goal.area_estimate_m2:.2f}m²")
    if plan is None:
        lines.append("  Plan:      NONE (no path or no goal)")
    else:
        lines.append(f"  Plan:      {len(plan.waypoints)} waypoints  "
                     f"{path_len:.2f} m total")
        lines.append(f"             end: ({plan.waypoints[-1][0]:+.2f}, "
                     f"{plan.waypoints[-1][1]:+.2f})")
    lines.append("─" * 64)
    ax4.text(0.02, 0.98, "\n".join(lines), family="monospace", fontsize=9,
             verticalalignment="top")

    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


# ── CLI driver ──────────────────────────────────────────────────────────────

def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--mock", action="store_true",
                   help="Use MockLidarSight (no hardware needed).")
    p.add_argument("--n", type=int, default=3,
                   help="Number of frames to render.")
    p.add_argument("--accumulate", type=int, default=5,
                   help="LiDAR frames to accumulate per render "
                        "(higher = denser cloud, slightly motion-smeared).")
    p.add_argument("--topic", default="rt/utlidar/cloud_livox_mid360")
    p.add_argument("--out", default="/tmp/plan_frames",
                   help="Output directory for rendered JPEGs.")
    p.add_argument("--clearance-m", type=float, default=DEFAULT_CLEARANCE_M,
                   help="Robot clearance for obstacle inflation, in metres.")
    p.add_argument("--cell-m", type=float, default=DEFAULT_CELL_M,
                   help="Occupancy grid cell size, in metres.")
    p.add_argument("--x-min", type=float, default=-1.0)
    p.add_argument("--x-max", type=float, default=5.0)
    p.add_argument("--y-min", type=float, default=-3.0)
    p.add_argument("--y-max", type=float, default=3.0)
    p.add_argument("--unknown-as-obstacle", action="store_true", default=False,
                   help="Treat UNKNOWN cells as hard-blocked (very conservative).")
    p.add_argument("--unknown-cost-multiplier", type=float, default=3.0,
                   help="Soft penalty for UNKNOWN cells in A* "
                        "(1.0 = no penalty, higher = prefer KNOWN-FREE routes).")
    p.add_argument("--goal-xy", type=float, nargs=2, default=None,
                   metavar=("X", "Y"),
                   help="Override pick_goal_table and plan directly to this "
                        "body-frame (x, y) target.  Useful when find_tables "
                        "is flaky or when testing specific path geometries.")
    p.add_argument("--start-clearing-m", type=float, default=0.80,
                   help="Radius of the circular free zone the planner carves "
                        "around the start (robot self-reflection envelope).")
    args = p.parse_args()

    os.makedirs(args.out, exist_ok=True)

    if args.mock:
        lidar: object = MockLidarSight()
    else:
        lidar = LidarSight.instance(topic=args.topic,
                                     accumulate_frames=args.accumulate,
                                     warmup_timeout_s=5.0)
        # Let the accumulator fill.
        time.sleep(max(0.1, args.accumulate * 0.12))

    bounds_x = (args.x_min, args.x_max)
    bounds_y = (args.y_min, args.y_max)

    for idx in range(args.n):
        cloud = lidar.latest_cloud()
        if cloud is None:
            print(f"[{idx}] no cloud yet — skipping")
            time.sleep(0.2)
            continue
        tables = lidar.find_tables()
        goal = pick_goal_table(tables)
        scene = build_scene_map(cloud, tables=tables,
                                x_bounds=bounds_x, y_bounds=bounds_y,
                                cell_m=args.cell_m)
        inflated = _inflate_obstacles(scene.grid, scene.cell_m,
                                       args.clearance_m,
                                       args.unknown_as_obstacle)
        plan = None
        # Decide the (x, y) goal: explicit override wins over picked table.
        goal_xy: Optional[tuple] = None
        if args.goal_xy is not None:
            goal_xy = (float(args.goal_xy[0]), float(args.goal_xy[1]))
        elif goal is not None:
            goal_xy = (goal.center_xyz[0], goal.center_xyz[1])
        if goal_xy is not None:
            plan = plan_path(
                scene,
                start_xy=(0.0, 0.0),
                goal_xy=goal_xy,
                clearance_m=args.clearance_m,
                unknown_as_obstacle=args.unknown_as_obstacle,
                unknown_cost_multiplier=args.unknown_cost_multiplier,
                start_clearing_radius_m=args.start_clearing_m,
            )

        print(f"[{idx}] {scene.summary()}")
        print(f"    goal: {None if goal is None else goal.center_xyz[:2]}")
        if plan is None:
            print(f"    plan: NONE")
        else:
            length = sum(
                ((plan.waypoints[k][0] - plan.waypoints[k - 1][0]) ** 2 +
                 (plan.waypoints[k][1] - plan.waypoints[k - 1][1]) ** 2) ** 0.5
                for k in range(1, len(plan.waypoints))
            )
            print(f"    plan: {len(plan.waypoints)} wp, {length:.2f} m, "
                  f"end={plan.waypoints[-1]}")

        out_path = os.path.join(args.out, f"plan_{idx:03d}.jpg")
        _render_frame(out_path, cloud.points, tables, goal, scene,
                      inflated, plan, idx, cloud.timestamp)
        print(f"    wrote {out_path}")

        if idx < args.n - 1:
            time.sleep(0.2)

    if not args.mock:
        lidar.shutdown()  # type: ignore[union-attr]
    return 0


if __name__ == "__main__":
    sys.exit(main())
