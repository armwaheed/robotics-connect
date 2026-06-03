#!/usr/bin/env python3
"""Capture LiDAR frames and render them as annotated JPEGs for offline review.

Used during bring-up and when investigating a tricky scene: grabs N live
frames (optionally accumulated), runs `find_tables()` on each, and
produces four views per frame:

  top  — XY (bird's-eye, +X forward, +Y left), z colour-coded
  side — XZ (profile, +X forward), y colour-coded
  front— YZ (how the table sits in front of the robot), x colour-coded
  3d   — isometric projection with detected tables highlighted

Each image overlays:
  * The robot's base marker (0, 0)
  * The estimated floor z and table-band envelope
  * Bounding boxes + centroids for every detected Table

Run on the robot:

    conda activate unitree_deploy
    cd /home/unitree/robotics-connect/lidar_sight
    python3 _capture_frames.py --n 5 --accumulate 3 --out /tmp/lidar_frames
"""
from __future__ import annotations

import argparse
import os
import time
from typing import List, Optional

import numpy as np

import matplotlib
matplotlib.use("Agg")  # headless — no display on the robot
import matplotlib.pyplot as plt

from lidar_sight import (
    LidarSight,
    MIN_TABLE_H_ABOVE_FLOOR_M,
    MAX_TABLE_H_ABOVE_FLOOR_M,
    Table,
    _estimate_floor_z,
    _find_horizontal_planes,
    _voxel_downsample,
)


def _scatter_zcolor(ax, xs, ys, zs, *, size=1.5, cmap="turbo", zlim=None,
                    title=None, xlabel=None, ylabel=None, equal=True):
    if zlim is None:
        vmin = float(np.percentile(zs, 1))
        vmax = float(np.percentile(zs, 99))
    else:
        vmin, vmax = zlim
    sc = ax.scatter(xs, ys, c=zs, s=size, cmap=cmap, vmin=vmin, vmax=vmax,
                    linewidths=0, alpha=0.9)
    if equal:
        ax.set_aspect("equal")
    if title:
        ax.set_title(title, fontsize=10)
    if xlabel:
        ax.set_xlabel(xlabel, fontsize=8)
    if ylabel:
        ax.set_ylabel(ylabel, fontsize=8)
    ax.grid(True, alpha=0.3, linewidth=0.3)
    ax.tick_params(labelsize=7)
    return sc


def _overlay_tables_top(ax, tables: List[Table]):
    for i, t in enumerate(tables):
        cx, cy, cz = t.center_xyz
        radius = max(0.15, (t.area_estimate_m2 / 3.14159) ** 0.5)
        c = plt.Circle((cx, cy), radius, fill=False, color="red",
                       linewidth=1.5, linestyle="-")
        ax.add_patch(c)
        ax.plot(cx, cy, "r+", markersize=10, markeredgewidth=1.5)
        ax.annotate(f"#{i} z={cz:+.2f}\n{t.area_estimate_m2:.2f}m²\nn={t.point_count}",
                    (cx, cy), textcoords="offset points", xytext=(8, 6),
                    fontsize=7, color="red",
                    bbox=dict(boxstyle="round,pad=0.2",
                              facecolor="white", edgecolor="red",
                              alpha=0.8, linewidth=0.5))


def _overlay_tables_side(ax, tables: List[Table], axis: str):
    """`axis`: which horizontal axis is plotted (x or y)."""
    for i, t in enumerate(tables):
        cx, cy, cz = t.center_xyz
        h = (cx if axis == "x" else cy)
        ax.plot(h, cz, "r+", markersize=10, markeredgewidth=1.5)
        ax.annotate(f"#{i}", (h, cz), textcoords="offset points",
                    xytext=(6, 4), fontsize=7, color="red")


def _render_frame(points: np.ndarray,
                  tables: List[Table],
                  floor_z: Optional[float],
                  planes: List[Table],
                  frame_idx: int,
                  out_dir: str,
                  zoom_forward_m: float = 2.5) -> str:
    fig = plt.figure(figsize=(15, 10))
    plane_summary = ", ".join(
        f"#{i}:z={p.center_xyz[2]:+.2f},{p.area_estimate_m2:.1f}m²"
        for i, p in enumerate(planes[:4])
    )
    fig.suptitle(
        f"LiDAR frame {frame_idx:03d}  "
        f"n={points.shape[0]}  "
        f"floor_z={floor_z if floor_z is None else round(floor_z, 3)}  "
        f"find_tables={len(tables)}  "
        f"horizontal_planes={len(planes)} [{plane_summary}]",
        fontsize=10,
    )

    p = points
    n_sample = min(p.shape[0], 60000)
    if p.shape[0] > n_sample:
        idx = np.random.default_rng(0).choice(p.shape[0], n_sample, replace=False)
        p = p[idx]

    ax_top = fig.add_subplot(2, 3, 1)
    _scatter_zcolor(ax_top, p[:, 0], p[:, 1], p[:, 2],
                    title="Top-down (XY)  +X fwd  +Y left  z colour",
                    xlabel="x forward [m]", ylabel="y left [m]")
    ax_top.plot(0, 0, "ko", markersize=6)
    ax_top.annotate("robot", (0, 0), textcoords="offset points",
                    xytext=(6, 6), fontsize=7)
    _overlay_tables_top(ax_top, planes)
    for r in (1.0, 2.0, 3.0):
        ax_top.add_patch(plt.Circle((0, 0), r, fill=False,
                                     color="gray", linestyle=":",
                                     linewidth=0.5))

    # Zoomed top-down on the forward corridor — easier to see the table
    ax_zoom = fig.add_subplot(2, 3, 2)
    zmask = ((p[:, 0] > -0.2) & (p[:, 0] < zoom_forward_m)
             & (np.abs(p[:, 1]) < 1.2))
    pz = p[zmask]
    if pz.shape[0] > 0:
        _scatter_zcolor(ax_zoom, pz[:, 0], pz[:, 1], pz[:, 2], size=3.0,
                        title=f"Top-down ZOOM  +X fwd  (x<{zoom_forward_m:.1f}m)",
                        xlabel="x forward [m]", ylabel="y left [m]")
    ax_zoom.plot(0, 0, "ko", markersize=5)
    ax_zoom.set_xlim(-0.2, zoom_forward_m)
    ax_zoom.set_ylim(-1.2, 1.2)
    _overlay_tables_top(ax_zoom, planes)

    ax_side = fig.add_subplot(2, 3, 3)
    _scatter_zcolor(ax_side, p[:, 0], p[:, 2], p[:, 1],
                    title="Side (XZ)  +X fwd  +Z up  y colour",
                    xlabel="x forward [m]", ylabel="z up [m]", equal=False)
    ax_side.axhline(0, color="black", linewidth=0.5, alpha=0.5)
    if floor_z is not None:
        ax_side.axhline(floor_z, color="green", linewidth=1.0,
                         linestyle="--", alpha=0.7, label=f"detected floor z={floor_z:.2f}")
        ax_side.axhspan(floor_z + MIN_TABLE_H_ABOVE_FLOOR_M,
                         floor_z + MAX_TABLE_H_ABOVE_FLOOR_M,
                         color="red", alpha=0.08, label="table band (above floor)")
        ax_side.legend(loc="upper right", fontsize=7, framealpha=0.8)
    # Overlay every horizontal plane centroid on the side view
    for i, pl in enumerate(planes):
        ax_side.axhline(pl.center_xyz[2], color="orange", linewidth=0.7,
                        linestyle=":", alpha=0.5)
        ax_side.annotate(f"plane#{i} z={pl.center_xyz[2]:+.2f}",
                         (0.02, pl.center_xyz[2]),
                         xycoords=("axes fraction", "data"),
                         fontsize=6, color="darkorange")
    _overlay_tables_side(ax_side, planes, axis="x")

    ax_front = fig.add_subplot(2, 3, 4)
    _scatter_zcolor(ax_front, p[:, 1], p[:, 2], p[:, 0],
                    title="Front (YZ)  +Y left  +Z up  x colour",
                    xlabel="y left [m]", ylabel="z up [m]", equal=False)
    ax_front.axhline(0, color="black", linewidth=0.5, alpha=0.5)
    if floor_z is not None:
        ax_front.axhline(floor_z, color="green", linewidth=1.0,
                          linestyle="--", alpha=0.7)
        ax_front.axhspan(floor_z + MIN_TABLE_H_ABOVE_FLOOR_M,
                          floor_z + MAX_TABLE_H_ABOVE_FLOOR_M,
                          color="red", alpha=0.08)
    _overlay_tables_side(ax_front, planes, axis="y")

    # Z histogram — lets us see every plane candidate at a glance
    ax_hist = fig.add_subplot(2, 3, 5)
    z_min = float(points[:, 2].min())
    z_max = float(points[:, 2].max())
    ax_hist.hist(points[:, 2], bins=100, range=(z_min, z_max),
                 orientation="horizontal", color="steelblue", alpha=0.8)
    ax_hist.set_title(
        f"Z histogram  z range [{z_min:+.2f}, {z_max:+.2f}]", fontsize=10)
    ax_hist.set_xlabel("count", fontsize=8)
    ax_hist.set_ylabel("z up [m]", fontsize=8)
    ax_hist.set_ylim(z_min - 0.05, z_max + 0.05)
    ax_hist.tick_params(labelsize=7)
    ax_hist.axhline(0, color="black", linewidth=0.5, alpha=0.6)
    if floor_z is not None:
        ax_hist.axhline(floor_z, color="green", linewidth=1.0,
                         linestyle="--", alpha=0.7,
                         label=f"detected floor {floor_z:.2f}")
    for i, pl in enumerate(planes):
        ax_hist.axhline(pl.center_xyz[2], color="orange", linewidth=0.7,
                         linestyle=":", alpha=0.6)
    ax_hist.legend(loc="upper right", fontsize=7)
    ax_hist.grid(True, alpha=0.3, linewidth=0.3)

    ax_3d = fig.add_subplot(2, 3, 6, projection="3d")
    # Subsample more aggressively for 3D (matplotlib chokes on >20 k).
    n3 = min(p.shape[0], 15000)
    if p.shape[0] > n3:
        i3 = np.random.default_rng(1).choice(p.shape[0], n3, replace=False)
        p3 = p[i3]
    else:
        p3 = p
    vmin = float(np.percentile(p3[:, 2], 1))
    vmax = float(np.percentile(p3[:, 2], 99))
    ax_3d.scatter(p3[:, 0], p3[:, 1], p3[:, 2], c=p3[:, 2],
                  s=0.8, cmap="turbo", vmin=vmin, vmax=vmax,
                  linewidths=0, alpha=0.8)
    ax_3d.set_title("3D  +X fwd  +Y left  +Z up", fontsize=10)
    ax_3d.set_xlabel("x", fontsize=7)
    ax_3d.set_ylabel("y", fontsize=7)
    ax_3d.set_zlabel("z", fontsize=7)
    ax_3d.tick_params(labelsize=6)
    for pl in planes:
        cx, cy, cz = pl.center_xyz
        ax_3d.scatter([cx], [cy], [cz], color="red", s=60, marker="+")
    ax_3d.view_init(elev=25, azim=-60)

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    out = os.path.join(out_dir, f"frame_{frame_idx:03d}.jpg")
    fig.savefig(out, dpi=110, format="jpg", bbox_inches="tight")
    plt.close(fig)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--topic", default="rt/utlidar/cloud_livox_mid360")
    ap.add_argument("--n", type=int, default=5,
                    help="Number of frames to capture+render")
    ap.add_argument("--accumulate", type=int, default=3,
                    help="Accumulator window (frames per render)")
    ap.add_argument("--warmup-s", type=float, default=1.0)
    ap.add_argument("--interval-s", type=float, default=0.5,
                    help="Delay between captures")
    ap.add_argument("--out", default="/tmp/lidar_frames")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    lidar = LidarSight.instance(topic=args.topic,
                                 accumulate_frames=args.accumulate)
    time.sleep(args.warmup_s)

    written = []
    for i in range(args.n):
        c = lidar.latest_cloud()
        if c is None:
            print(f"[{i}] no cloud yet")
            time.sleep(args.interval_s)
            continue
        p = c.points
        ds = _voxel_downsample(p, 0.05)
        fz = _estimate_floor_z(ds)
        tables = lidar.find_tables()
        planes = lidar.find_horizontal_planes()
        print(f"[{i}] n={p.shape[0]:6d} floor_z={fz} find_tables={len(tables)} planes={len(planes)}")
        for j, t in enumerate(planes):
            print(f"    plane#{j} center={t.center_xyz} area={t.area_estimate_m2:.2f}m² "
                  f"conf={t.confidence:.2f} n={t.point_count}")
        path = _render_frame(p, tables, fz, planes, i, args.out)
        written.append(path)
        time.sleep(args.interval_s)

    lidar.shutdown()
    print(f"wrote {len(written)} frames to {args.out}")
    for w in written:
        print(f"  {w}")


if __name__ == "__main__":
    main()
