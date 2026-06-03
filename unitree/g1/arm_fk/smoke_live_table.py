#!/usr/bin/env python3
"""
smoke_live_table.py — live diagnostic for DepthCameraSight.table_plane_z()

Queries the live head camera once, prints the body-Z histogram + all
percentiles, and reports what `table_plane_z()` picked as the nearest
surface.  Use this on the robot to verify which surface the camera is
selecting before (and after) a live reach run.

Usage (on the robot):

    source ~/miniconda3/etc/profile.d/conda.sh
    conda activate unitree_deploy
    export LD_PRELOAD=/home/unitree/miniconda3/envs/unitree_deploy/lib/libgomp.so.1
    export CYCLONEDDS_URI=file:///home/unitree/cyclonedds.xml
    cd /home/unitree/robotics-connect
    source depth_camera_sight/setup_env.sh
    python arm_fk/smoke_live_table.py

Reads one frame, prints a small histogram, and exits.  No arm motion,
no DDS writes.
"""
from __future__ import annotations

import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "depth_camera_sight"))

from depth_camera_sight import DepthCameraSight  # noqa: E402


def _hbar(count: int, max_count: int, width: int = 40) -> str:
    if max_count <= 0:
        return ""
    n = int(round(width * count / max_count))
    return "█" * n


def main() -> int:
    cam = DepthCameraSight.instance()
    print("[smoke_live_table] waiting 2 s for depth pipeline to warm up...")
    time.sleep(2.0)

    diag = cam.table_plane_z_diag()
    if diag is None:
        print("[smoke_live_table] FAIL: no valid depth frame available")
        cam.shutdown()
        return 1

    print()
    print(f"  n_samples      = {diag['n_samples']}")
    print(f"  body_z range   = [{diag['min_z']:+.3f}, {diag['max_z']:+.3f}] m")
    print(f"  body_z median  = {diag['median_z']:+.3f} m")
    print(f"  percentiles    p25={diag['p25']:+.3f}  p50={diag['p50']:+.3f}  "
          f"p75={diag['p75']:+.3f}  p90={diag['p90']:+.3f}  "
          f"p95={diag['p95']:+.3f}  p99={diag['p99']:+.3f}")
    print()
    print(f"  table_plane_z() = {diag['table_plane_z']:+.3f} m  "
          f"→ nearest surface is {-diag['table_plane_z']*100:.1f} cm below camera mount")
    print()
    print("  Histogram (body-Z, nearest-camera at the TOP, empty bins hidden):")
    print("  " + "-" * 60)
    counts = diag["bin_counts"]
    edges = diag["bin_edges"]
    max_count = max(counts) if counts else 0
    picked_z = diag["table_plane_z"]
    # Print top-down so nearest-to-camera is at the top (matches how
    # the picker walks the bins).  Skip empty bins for brevity — a
    # typical indoor scene only has a handful of non-empty bins
    # (floor + table + maybe a pillow).
    for i in range(len(counts) - 1, -1, -1):
        c = counts[i]
        if c == 0:
            continue
        z_lo = edges[i]
        z_hi = edges[i + 1]
        marker = "  <-- picked" if (picked_z is not None
                                     and z_lo <= picked_z <= z_hi) else ""
        print(f"  [{z_lo:+.3f}, {z_hi:+.3f}] m  {c:5d}  {_hbar(c, max_count)}{marker}")
    print("  " + "-" * 60)
    print()

    cam.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
