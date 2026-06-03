#!/usr/bin/env python3
"""
Calibrate CAMERA_TILT_DEG_DEFAULT from a live depth frame.

Method: the floor plane is a known reference.  We know the camera mount
height above the floor H_cam (user-measured).  Every valid depth pixel
that is actually *on the floor* must, after back-projection and rotation
into body frame, have body_z = -H_cam.  So the tilt `t` must satisfy
    cy·cos(t) + cz·sin(t) = H_cam
for every floor pixel (cy, cz).  With many floor pixels this is a
(massively) overdetermined linear system in (cos t, sin t).  Solve it
by least squares, renormalise, take atan2.

Outlier handling: depth pixels are a mix of floor + whatever furniture
and walls are in the scene, plus saturated "no reading" pixels at
~65.535 m.  We strip those, then fit iteratively with a simple
residual-based reject loop (a lightweight RANSAC substitute).
"""
from __future__ import annotations
import argparse
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from depth_camera_sight import DepthCameraSight  # noqa: E402


def body_z_for_all(cy: np.ndarray, cz: np.ndarray, tilt_deg: float) -> np.ndarray:
    t = math.radians(tilt_deg)
    return -cy * math.cos(t) - cz * math.sin(t)


def fit_tilt(cy: np.ndarray, cz: np.ndarray, H_cam: float) -> tuple[float, float, int]:
    """Least-squares fit with a residual-rejection inner loop.

    Returns (tilt_deg, residual_rms_m, n_inliers).
    """
    # A @ [a; b] = H   where a = cos(t), b = sin(t)
    # A rows are [cy_i, cz_i].  Rebuild H as a constant vector.
    A = np.stack([cy, cz], axis=1).astype(np.float64)
    b_rhs = np.full(A.shape[0], H_cam, dtype=np.float64)

    mask = np.ones(A.shape[0], dtype=bool)
    for _ in range(5):
        sol, *_ = np.linalg.lstsq(A[mask], b_rhs[mask], rcond=None)
        a, bs = sol
        norm = math.hypot(a, bs)
        a /= norm
        bs /= norm
        t_deg = math.degrees(math.atan2(bs, a))
        residuals = np.abs(A @ np.array([a, bs]) - H_cam)
        # Keep the inner 70% of residuals under the current tilt.
        thresh = np.percentile(residuals[mask], 70)
        new_mask = residuals <= thresh
        if new_mask.sum() < 500:  # don't over-prune
            break
        if np.array_equal(new_mask, mask):
            break
        mask = new_mask
    rms = float(math.sqrt(float((residuals[mask] ** 2).mean())))
    return t_deg, rms, int(mask.sum())


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--camera-height-m", type=float, required=True,
                   help="Measured vertical distance from the floor to the D435i lens centre, in meters")
    p.add_argument("--bottle-forward-m", type=float, default=None,
                   help="Optional: measured forward distance from the robot's body X axis to a target (e.g. bottle base) in meters. Used for cross-check only.")
    p.add_argument("--max-depth-m", type=float, default=4.0,
                   help="Reject depth pixels beyond this range (masks out the 65.535m saturation and the far wall)")
    p.add_argument("--central-crop", type=float, default=0.6,
                   help="Fraction of the image centre to keep for floor fitting (reject the edges which may contain walls / furniture)")
    p.add_argument("--warmup-s", type=float, default=3.0)
    args = p.parse_args()

    cam = DepthCameraSight.instance(warmup_timeout_s=5.0)
    t_end = time.monotonic() + args.warmup_s
    while time.monotonic() < t_end:
        f = cam.latest()
        if f.depth_m is not None and (f.depth_m > 0).sum() > 10000:
            break
        time.sleep(0.1)
    frame = cam.latest()
    if frame.depth_m is None:
        print("FAIL: no depth frame", file=sys.stderr)
        return 1

    depth = frame.depth_m
    intr = frame.depth_intrinsics
    h, w = depth.shape
    fx, fy = intr["fx"], intr["fy"]
    cx, cy_intr = intr["cx"], intr["cy"]

    # Central crop — edges commonly catch walls/furniture we don't want in the fit.
    cw = int(w * args.central_crop)
    chh = int(h * args.central_crop)
    u0 = (w - cw) // 2
    v0 = (h - chh) // 2
    u1 = u0 + cw
    v1 = v0 + chh
    crop = depth[v0:v1, u0:u1]

    # Valid + in-range mask.
    valid = (crop > 0.1) & (crop < args.max_depth_m)
    vs, us = np.where(valid)
    # Convert back to full-image pixel coords for intrinsics math.
    us_full = us + u0
    vs_full = vs + v0
    ds = crop[vs, us]
    # Camera-frame coordinates (float32).
    cam_x = (us_full - cx) * ds / fx
    cam_y = (vs_full - cy_intr) * ds / fy
    cam_z = ds

    print(f"pixels used for fit: {len(ds)} "
          f"(central {int(args.central_crop*100)}% crop, "
          f"{args.max_depth_m}m max range)")

    tilt_deg, rms, n_in = fit_tilt(cam_y, cam_z, args.camera_height_m)
    print(f"\n=== FLOOR-PLANE FIT (constrained: H = user-measured) ===")
    print(f"  tilt_deg   = {tilt_deg:.3f}°")
    print(f"  residual   = {rms*100:.2f} cm RMS on {n_in} inlier pixels")
    print(f"  camera H   = {args.camera_height_m:.4f} m (user-measured)")
    bz = body_z_for_all(cam_y, cam_z, tilt_deg)
    print(f"  body_z med = {float(np.median(bz)):.4f} m  (target {-args.camera_height_m:.4f})")
    print(f"  body_z std = {float(np.std(bz))*100:.2f} cm")

    # ── Unconstrained fit: solve for (tilt, H) jointly via plane SVD ──
    # Stack camera-frame points, fit the best plane.  The plane normal in
    # camera frame is related to the floor normal in body frame ([0,0,1])
    # by our rotation.  Specifically, body z-axis in camera frame is
    # (0, -cos(t), -sin(t)), so the plane we fit should have that normal.
    pts = np.stack([cam_x, cam_y, cam_z], axis=1).astype(np.float64)
    # Trim to the inner-residual inliers from the constrained fit — same
    # mask semantics, just computed in the cam frame now.
    resid = np.abs(cam_y * math.cos(math.radians(tilt_deg))
                   + cam_z * math.sin(math.radians(tilt_deg))
                   - args.camera_height_m)
    keep = resid <= np.percentile(resid, 70)
    pts_k = pts[keep]
    centroid = pts_k.mean(axis=0)
    _, _, vt = np.linalg.svd(pts_k - centroid, full_matrices=False)
    normal = vt[-1]   # smallest singular vector = plane normal in cam frame
    # Orient so normal roughly points "up" (against cam +Y, which is cam-down).
    if normal[1] > 0:
        normal = -normal
    # normal should equal (0, -cos t, -sin t) for the floor-up direction.
    # → cos(t) = -ny, sin(t) = -nz
    a_free = -float(normal[1])
    b_free = -float(normal[2])
    norm = math.hypot(a_free, b_free)
    a_free /= norm
    b_free /= norm
    tilt_free_deg = math.degrees(math.atan2(b_free, a_free))
    # Recover H from the plane equation n · p = d, with d = n · centroid.
    d_plane = float(normal @ centroid)
    # n · centroid should equal (-cos t · cy_c - sin t · cz_c) = -body_z_c = H
    H_free = -d_plane  # since normal was flipped to be "up"
    rms_free = float(np.sqrt(((pts_k - centroid) @ normal).var()))
    print(f"\n=== PLANE-SVD FIT (unconstrained: tilt AND H both free) ===")
    print(f"  tilt_deg   = {tilt_free_deg:.3f}°")
    print(f"  camera H   = {H_free:.4f} m  (user said {args.camera_height_m:.4f})")
    print(f"  H delta    = {(H_free - args.camera_height_m)*100:+.2f} cm")
    print(f"  plane RMS  = {rms_free*100:.2f} cm")

    # Cross-check against a known forward distance if provided.
    # Use the unconstrained-fit tilt + H for the cross-check — it's the
    # strictly more accurate calibration.
    if args.bottle_forward_m is not None:
        t = math.radians(tilt_free_deg)
        ct, st = math.cos(t), math.sin(t)
        bx_all = -cam_y * st + cam_z * ct
        by_all = -cam_x
        bz_all = -cam_y * ct - cam_z * st
        target_x = args.bottle_forward_m
        # Select a neighbourhood of depth pixels that land near the
        # bottle in the (x, y) plane.
        near = (np.abs(bx_all - target_x) < 0.08) & (np.abs(by_all) < 0.08)
        if near.sum() < 5:
            print("\n=== BOTTLE CROSS-CHECK: not enough pixels in neighbourhood ===")
        else:
            # The bottle base is the MINIMUM body_z (most negative) in the
            # neighbourhood — i.e. the pixel closest to the floor plane.
            base_idx = np.where(near)[0][int(np.argmin(bz_all[near]))]
            # The bottle top is the MAXIMUM body_z in the neighbourhood.
            top_idx  = np.where(near)[0][int(np.argmax(bz_all[near]))]
            base = (float(bx_all[base_idx]), float(by_all[base_idx]), float(bz_all[base_idx]))
            top  = (float(bx_all[top_idx]),  float(by_all[top_idx]),  float(bz_all[top_idx]))
            print(f"\n=== BOTTLE CROSS-CHECK (free-fit tilt={tilt_free_deg:.3f}°, H={H_free:.4f}m) ===")
            print(f"  {near.sum()} depth pixels in the bottle neighbourhood")
            print(f"  base pixel body = ({base[0]:+.3f}, {base[1]:+.3f}, {base[2]:+.3f})")
            print(f"  top  pixel body = ({top[0]:+.3f}, {top[1]:+.3f}, {top[2]:+.3f})")
            print(f"  bottle height   = {(top[2]-base[2])*100:.1f} cm")
            # Gates: the base's forward distance should be within ±3 cm
            # of the user-measured value; the base's body_z should be within
            # ±3 cm of -H_free (the floor).
            dx = abs(base[0] - target_x)
            dz = abs(base[2] - (-H_free))
            gate = 3.0
            print(f"  base forward err = {dx*100:.1f} cm   base z-vs-floor err = {dz*100:.1f} cm")
            print(f"  {'PASS' if (dx < gate/100 and dz < gate/100) else 'FAIL'} ±{gate:.0f} cm gate")

    cam.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
