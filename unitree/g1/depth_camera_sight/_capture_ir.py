#!/usr/bin/env python3
"""Capture raw left + right IR frames from the D435i and save as PNGs.

Usage (on the robot):
    source setup_env.sh
    conda activate unitree_deploy
    python _capture_ir.py [--out /tmp/ir_capture] [--frames 5]

Produces:
    <out>/ir_left_000.png   — left IR sensor (this is the depth reference frame)
    <out>/ir_right_000.png  — right IR sensor
    <out>/depth_000.png     — depth colourmap (for visual cross-reference)

The IR streams show what the stereo matcher physically sees.  Useful for:
  - Debugging depth holes (areas where the D435i can't correlate L/R)
  - Verifying that both IR sensors have the object in view (objects at
    the extreme edge of one sensor's FOV may be invisible to the other,
    producing no depth at that pixel)
  - Understanding the depth camera's actual FOV vs the RGB camera

The left IR frame IS the coordinate frame that pyrealsense2's depth
stream is projected into.  So "pixel (u, v) in the left IR image"
corresponds to "pixel (u, v) in the depth frame" geometrically.
"""
import argparse
import os
import time

import numpy as np


def main():
    p = argparse.ArgumentParser(description="Capture D435i IR + depth frames")
    p.add_argument("--out", default="/tmp/ir_capture",
                   help="Output directory (created if absent)")
    p.add_argument("--frames", type=int, default=5,
                   help="Number of frame sets to capture")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=int, default=30)
    args = p.parse_args()

    import pyrealsense2 as rs

    os.makedirs(args.out, exist_ok=True)

    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)
    cfg.enable_stream(rs.stream.infrared, 1, args.width, args.height, rs.format.y8, args.fps)
    cfg.enable_stream(rs.stream.infrared, 2, args.width, args.height, rs.format.y8, args.fps)
    pipeline.start(cfg)

    print(f"Capturing {args.frames} frame sets to {args.out}/")
    time.sleep(0.5)

    try:
        for i in range(args.frames):
            frames = pipeline.wait_for_frames(timeout_ms=2000)
            ir_left = frames.get_infrared_frame(1)
            ir_right = frames.get_infrared_frame(2)
            depth = frames.get_depth_frame()

            if ir_left:
                arr = np.asanyarray(ir_left.get_data())
                path = os.path.join(args.out, f"ir_left_{i:03d}.png")
                _save_gray(arr, path)
                print(f"  [{i}] ir_left  {arr.shape}  saved → {path}")

            if ir_right:
                arr = np.asanyarray(ir_right.get_data())
                path = os.path.join(args.out, f"ir_right_{i:03d}.png")
                _save_gray(arr, path)
                print(f"  [{i}] ir_right {arr.shape}  saved → {path}")

            if depth:
                arr = np.asanyarray(depth.get_data())
                path = os.path.join(args.out, f"depth_{i:03d}.png")
                _save_depth_colormap(arr, path)
                print(f"  [{i}] depth    {arr.shape}  saved → {path}")

            time.sleep(0.2)
    finally:
        pipeline.stop()

    print(f"\nDone. {args.frames} frame sets in {args.out}/")


def _save_gray(arr: np.ndarray, path: str):
    """Save a uint8 grayscale array as a PNG without cv2."""
    try:
        import cv2
        cv2.imwrite(path, arr)
    except ImportError:
        from PIL import Image
        Image.fromarray(arr, mode="L").save(path)


def _save_depth_colormap(arr: np.ndarray, path: str):
    """Save a uint16 depth array as a false-colour PNG."""
    try:
        import cv2
        norm = cv2.normalize(arr, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        cm = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
        cv2.imwrite(path, cm)
    except ImportError:
        clipped = np.clip(arr.astype(np.float32) / max(arr.max(), 1), 0, 1)
        gray = (clipped * 255).astype(np.uint8)
        _save_gray(gray, path)


if __name__ == "__main__":
    main()
