#!/usr/bin/env python3
"""
One-shot sanity check for Depth Camera Sight on the G1.

Run this from a future Claude Code session to confirm the camera is alive:

    ssh unitree@192.168.123.164
    export LD_LIBRARY_PATH=/home/unitree/librealsense/build:$LD_LIBRARY_PATH
    export PYTHONPATH=/home/unitree/librealsense/build/wrappers/python:$PYTHONPATH
    export LD_PRELOAD=/home/unitree/miniconda3/envs/unitree_deploy/lib/libgomp.so.1
    /home/unitree/miniconda3/envs/unitree_deploy/bin/python \
        /home/unitree/robotics-connect/depth_camera_sight/_diag_camera_test.py

Expected output: one line per stream summarising frame shape, valid
depth pixel count, depth range, and rgb mean per channel.  Exits
non-zero if either stream fails to warm up within 5 s.
"""
import sys
import os
import time

# Make the package importable regardless of where this is run from.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from depth_camera_sight import DepthCameraSight  # noqa: E402
import numpy as np  # noqa: E402


def main() -> int:
    try:
        cam = DepthCameraSight.instance()
    except RuntimeError as e:
        print(f"FAIL: {e}", file=sys.stderr)
        return 1

    # The singleton returns as soon as EITHER stream warms up.  Give the
    # slower stream (Unitree DDS RGB, ~1 s first-frame latency) a chance
    # to catch up before we judge success.
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        frame = cam.latest()
        if frame.rgb is not None and frame.depth_m is not None:
            break
        time.sleep(0.1)
    frame = cam.latest()
    ok = True

    if frame.rgb is None:
        print("FAIL: rgb stream not warm", file=sys.stderr)
        ok = False
    else:
        mean = frame.rgb.reshape(-1, 3).mean(axis=0)
        print(f"rgb   ok  shape={frame.rgb.shape}  mean_rgb={mean.astype(int).tolist()}")

    if frame.depth_m is None:
        print("FAIL: depth stream not warm", file=sys.stderr)
        ok = False
    else:
        valid = frame.depth_m > 0
        n = int(valid.sum())
        if n == 0:
            print("FAIL: depth has zero valid pixels", file=sys.stderr)
            ok = False
        else:
            mn = float(frame.depth_m[valid].min())
            mx = float(frame.depth_m[valid].max())
            mean = float(frame.depth_m[valid].mean())
            print(f"depth ok  shape={frame.depth_m.shape}  "
                  f"valid={n}/{frame.depth_m.size}  "
                  f"min={mn:.3f}m  mean={mean:.3f}m  max={mx:.3f}m")
            print(f"depth intrinsics: {frame.depth_intrinsics}")

    center_u = frame.depth_intrinsics["width"] // 2 if frame.depth_intrinsics else 0
    center_v = frame.depth_intrinsics["height"] // 2 if frame.depth_intrinsics else 0
    body = cam.pixel_to_body_xyz(center_u, center_v)
    print(f"center pixel_to_body_xyz: {body}  (tilt={frame.tilt_deg:.1f}°)")

    cam.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
