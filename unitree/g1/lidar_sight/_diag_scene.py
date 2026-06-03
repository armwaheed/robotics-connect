#!/usr/bin/env python3
"""Interactive scene diagnostic for lidar_sight bring-up.

Subscribes to the live LiDAR, optionally accumulates a few frames, and
prints a body-frame z histogram + floor estimate + table search results at
several band settings.  The tool is NOT called from any
hot path — it's a manual on-robot sanity check for:

  * Verifying the LiDAR is publishing and the mount transform is correct.
  * Sanity-checking the `_estimate_floor_z` heuristic against whatever
    posture the robot is currently in (standing / seated-on-cart /
    collapsed all produce different z ranges).
  * Running `find_tables()` at a few non-default bands to understand
    what the scene contains before tuning the production constants.

Run on the robot:

    conda activate unitree_deploy
    cd /home/unitree/robotics-connect/lidar_sight
    python3 _diag_scene.py --accumulate 5
"""
from __future__ import annotations

import argparse
import time

import numpy as np

from lidar_sight import (
    LidarSight,
    _estimate_floor_z,
    _find_tables_from_cloud,
    _voxel_downsample,
)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--topic", default="rt/utlidar/cloud_livox_mid360")
    ap.add_argument("--accumulate", type=int, default=3)
    ap.add_argument("--warmup-sleep-s", type=float, default=1.0)
    args = ap.parse_args()

    lidar = LidarSight.instance(topic=args.topic,
                                 accumulate_frames=args.accumulate)
    time.sleep(args.warmup_sleep_s)
    c = lidar.latest_cloud()
    if c is None:
        raise SystemExit("no cloud received")
    p = c.points
    print(f"points: {p.shape}")
    print(f"x range: {p[:,0].min():+.2f} .. {p[:,0].max():+.2f}")
    print(f"y range: {p[:,1].min():+.2f} .. {p[:,1].max():+.2f}")
    print(f"z range: {p[:,2].min():+.2f} .. {p[:,2].max():+.2f}")

    ds = _voxel_downsample(p, 0.05)
    fz = _estimate_floor_z(ds)
    print(f"downsampled: {ds.shape}")
    print(f"detected floor z: {fz}")

    # Z histogram (0.10 m bins over the observed range)
    zs = p[:, 2]
    lo, hi = float(zs.min()), float(zs.max())
    bin_w = 0.10
    import math
    nb = max(3, int(math.ceil((hi - lo) / bin_w)))
    hist, edges = np.histogram(zs, bins=nb)
    print("body-z histogram (0.10 m bins):")
    mx = int(hist.max()) or 1
    for ct, a, b in zip(hist, edges[:-1], edges[1:]):
        bar = "#" * int(40 * ct / mx)
        print(f"  [{a:+.2f},{b:+.2f}]: {int(ct):6d} {bar}")

    # Table search at a few bands so a human can see what the scene contains
    probes = [(0.10, 0.40), (0.40, 0.70), (0.55, 1.15), (0.80, 1.30)]
    for (lo_, hi_) in probes:
        tabs = _find_tables_from_cloud(
            p, 0.05, lo_, hi_, 0.02, 40, 0.10, 0.05,
        )
        print(f"band above-floor=[{lo_:.2f},{hi_:.2f}]: {len(tabs)} candidate(s)")
        for t in tabs:
            print(f"   center={t.center_xyz} area={t.area_estimate_m2:.2f}m² "
                  f"conf={t.confidence:.2f} n={t.point_count}")

    lidar.shutdown()


if __name__ == "__main__":
    main()
