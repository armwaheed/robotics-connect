# Depth Camera Sight

Singleton service that exposes the G1 head Intel RealSense D435i as
import-and-call Python.

> **RGB → GPU sidecar.** The RGB frames this module publishes can feed
> DINOv2 ViT-S/14. The encoder
> runs inside the GPU vision sidecar container
> ([`vision_sidecar/`](../vision_sidecar/README.md)) so host
> `unitree_deploy` stays CPU-torch-only. If you're wiring a *new* model
> downstream of these frames and it needs accelerated inference, send
> it through the same sidecar pattern rather than installing GPU torch
> into the host env. `--cpu` on every consumer forces the in-process
> fallback.

> **Body-frame geometry is shared with `arm_fk`.**
> `pixel_to_body_xyz` and `table_plane_z` both report in the same
> body frame (origin at the d435 camera mount, +X forward, +Y left,
> +Z up) that [`arm_fk/`](../arm_fk/README.md) uses for
> `forward_body_frame` / `palm_xyz`. That means you can compare
> `arm_fk.palm_xyz(arm_q)["L_palm"][2]` directly against
> `camera.table_plane_z()` — both numbers are metres relative to the
> same origin. The arm-reach handoff trigger is built on
> exactly this pairing. If you add a new camera geometry helper to
> this module, keep the frame identical so arm_fk can consume it
> without translation gymnastics.

```python
from depth_camera_sight import DepthCameraSight

cam = DepthCameraSight.instance()
frame = cam.latest()
if frame.depth_m is not None:
    xy = cam.pixel_to_table_xy(u=320, v=240)  # (x_forward_m, y_left_m)
cam.shutdown()
```

## Camera-mount geometry: RGB vs depth vs IR

The G1 EDU's head module contains an Intel D435i (depth + IR stereo +
RGB), plus a separate Unitree RGB camera. They are mounted under the
robot's "brow" at slightly different positions:

| Sensor | Mount position | Used by |
|---|---|---|
| **Unitree RGB** (VideoClient DDS) | ~1 cm left of center | DINOv2 RGB consumers, phone-app video preview |
| **D435i left IR** (pyrealsense2 depth reference) | ~center (left IR sits ~8.5 mm left of the module midline) | `object_xyz_on_table_m`, `table_plane_z`, all body-frame geometry |
| **D435i right IR** | ~55 mm right of left IR | stereo matching only (not exposed as a separate stream by this module) |
| **D435i RGB** | between the two IR sensors | not used — blocked by `videohub_pc4` |

All body-frame geometry (`pixel_to_body_xyz`, `object_xyz_on_table_m`,
`table_plane_z`) is in the **D435i left-IR sensor's frame**, which is
what `pyrealsense2` projects the depth stream into by default.  The
URDF's `d435_link` is calibrated to the physical D435i module mount.
The ~8.5 mm offset between the left-IR principal point and the module
midline is absorbed into the downstream proportional calibration
constants and does not need explicit correction.

The Unitree RGB camera's left-of-center offset causes objects to
appear shifted to the right in the phone-app preview relative to their
depth-frame position.  This is cosmetic — no code path compares RGB
pixel positions against depth body-frame coordinates.

**IR capture:** `_capture_ir.py` saves raw left and right IR frames
as PNGs alongside the depth frame for debugging depth holes and
verifying the stereo FOV.

## Hard constraint: the head camera is angled downward

The G1 EDU's head D435i is mounted pointing down. It sees the floor,
the near table surface, and any object resting on that table. It
**does not** see standing humans at conversational range or the broader
room. Every consumer must be designed around that geometry — the
downward-only view is the reason LiDAR Sight Mode is a separate,
complementary service.

## Files

| File | Purpose |
|---|---|
| `depth_camera_sight.py`   | Singleton + mock + geometry helpers + CLI diag |
| `_diag_camera_test.py`    | One-shot sanity check for future sessions |
| `_calibrate_tilt.py`      | Floor-plane fit to calibrate the downward tilt angle |
| `_capture_ir.py`          | Save raw left + right IR frames as PNGs (debug stereo + depth holes) |
| `install_pyrealsense2.sh` | Build pyrealsense2 from source (customer-deploy installer) |
| `setup_env.sh`            | Sourceable env bootstrap (`LD_LIBRARY_PATH`, `PYTHONPATH`, `LD_PRELOAD`) |
| `README.md`               | This file |

No `__init__.py` on purpose — the package imports the main module by
path, matching the layout of the sibling control-mode packages.

## Architecture: hybrid depth + RGB

The obvious approach would be "open both color and depth with
librealsense and call it a day". **On this robot that does not work.**

`videohub_pc4`, the Unitree root service that feeds the phone app's
video preview, holds `/dev/video4` (the RGB V4L2 node) open at all
times. Any second process trying to `VIDIOC_S_FMT` on that node
immediately hits `Device or resource busy`. We **must not** kill
`videohub_pc4` — the operator relies on it for remote video monitoring
over the Unitree phone app.

What actually works:

| Stream | Source | Rate | Resolution |
|---|---|---|---|
| depth  | `pyrealsense2` (direct librealsense on `/dev/video2`-ish) | 30 Hz | 640×480 float32 m |
| rgb    | `unitree_sdk2py.go2.video.VideoClient` over DDS (Unitree video hub)     | ~15 Hz | 1920×1080 uint8 RGB |

Two capture threads, one most-recent-frame slot per stream, no locking
between them. Consumers get a `SightFrame` that carries both with
independent timestamps.

The two streams are **not pixel-aligned** at this layer. Geometry
helpers (`pixel_to_body_xyz`, `pixel_to_table_xy`) operate on **depth**
pixels using the depth-sensor intrinsics. RGB is exposed as-is for
holistic features (e.g. DINOv2 in ACT-lite) where
pixel-precise alignment is irrelevant. If a future consumer genuinely
needs "find object in RGB → look up depth at that pixel", the solution
is: (a) switch depth to `848×480` to match the RGB sensor's aspect
better, (b) run an offline calibration of the depth→color extrinsic
(librealsense exposes it at stream setup time), and (c) do the
projection in the consumer. We deliberately did not build that in v1.

## Installation path

There are three pieces to install: (1) the `librealsense` C++ library,
(2) custom-built pyrealsense2 Python bindings, (3) the Robotics Connect repo
with `depth_camera_sight/`. Only (2) has to be *built* — (1) comes from
apt, and (3) is a `git clone`.

The **canonical install prefix** on this robot is:

```
/home/unitree/librealsense/       # librealsense source + build artifacts (not in git)
/home/unitree/robotics-connect/   # Robotics Connect repo (editable pip install, in git)
```

`/home/unitree/librealsense/` is a **deliberate choice** — it's outside
the repo, so git stays clean, but on a path that survives re-flashes
of the code (the `/home/unitree/` user directory is preserved). Do not
move it without updating `setup_env.sh`'s `LIBRS_PREFIX` default.

### Fresh install (re-flashed robot, or customer deploy)

```bash
# Dependencies are already on the G1 EDU's stock image: gcc, g++,
# cmake, make, libusb-1.0-0-dev, libudev-dev, conda's python3-dev.
# On a fresh Ubuntu 20.04 aarch64 box you'd apt-install them first.

cd /home/unitree/robotics-connect/depth_camera_sight
bash install_pyrealsense2.sh              # ~10 min, idempotent
source setup_env.sh                        # sets LD_LIBRARY_PATH, PYTHONPATH, LD_PRELOAD
export CYCLONEDDS_URI=file:///home/unitree/cyclonedds.xml   # robot-side DDS config (pair with LD_PRELOAD)
/home/unitree/miniconda3/envs/unitree_deploy/bin/python _diag_camera_test.py
```

**The installer is idempotent.** Re-running it is a no-op if the
build is already present and imports successfully. Re-running it
after a failed build or a Python upgrade rebuilds cleanly.

### Customer-deploy packaging (future)

The installer intentionally does not `apt install` anything — it only
builds against libraries that are already on the target image. That
keeps it safe to run unattended and free of sudo prompts. The current
cleanest customer-deploy shape is:

1. `git clone` the Robotics Connect repo to the target robot
2. Run `bash robotics-connect/depth_camera_sight/install_pyrealsense2.sh`
3. Add `source .../setup_env.sh` to the robot's startup shell config
4. Done — `depth_camera_sight.py` is importable in-process from any
   consumer in `unitree_deploy`

If the target robot has no internet, pre-download the librealsense
source tarball and set `SRC_TARBALL=/path/to/librealsense-v2.50.0.tar.gz`
when invoking the installer. It will extract the tarball instead of
cloning.

### Env vars (what setup_env.sh actually sets)

```bash
export LD_LIBRARY_PATH=/home/unitree/librealsense/build:$LD_LIBRARY_PATH
export PYTHONPATH=/home/unitree/librealsense/build/wrappers/python:$PYTHONPATH
export LD_PRELOAD=/home/unitree/miniconda3/envs/unitree_deploy/lib/libgomp.so.1
```

`setup_env.sh` does **not** export `CYCLONEDDS_URI` — always pair your
invocation with the robot's Cyclone DDS config when talking to the
Unitree DDS stack (e.g. via `VideoClient` in this module's RGB path):

```bash
export CYCLONEDDS_URI=file:///home/unitree/cyclonedds.xml
```

This tells `cyclonedds` to use the robot's unicast-routed config instead
of the default multicast discovery, which is what the rest of the
Unitree services on this machine expect.

Conda env: **`unitree_deploy`** (Python 3.10.12). The bindings were
built specifically against this interpreter — they will not load into
the system Python 3.8.

Smoke test (after sourcing `setup_env.sh`):

```bash
/home/unitree/miniconda3/envs/unitree_deploy/bin/python \
    /home/unitree/robotics-connect/depth_camera_sight/_diag_camera_test.py
```

Expected output (within ~1 s of start):

```
rgb   ok  shape=(1080, 1920, 3)  mean_rgb=[...]
depth ok  shape=(480, 640)  valid=228xxx/307200  min=0.18m  mean=~1.5m  max=65.535m
depth intrinsics: {'fx': 392.13..., 'fy': 392.13..., 'cx': 320.9..., 'cy': 241.8..., ...}
center pixel_to_body_xyz: [1.33 0.00 0.77]  (tilt=30.0°)
```

**Known quirk:** depth pixels that hit sensor out-of-range saturate at
`65.535 m` (raw uint16 max × depth_scale 0.001). Consumers must mask
`(depth_m > 0) & (depth_m < MAX_REASONABLE_RANGE)` before using. 0 is
"no measurement" in the librealsense convention; the 65.535 saturation
is specific to how this pipeline serves out-of-range pixels.

## Building pyrealsense2 from source (one-time, already done)

Keep this section for when the robot is re-flashed or the build is
lost. The librealsense version must match the installed
`librealsense2.so` under `/opt/ros/noetic/lib/aarch64-linux-gnu/` — on
this robot that is **v2.50.0**.

```bash
# From a machine with internet:
git clone -b v2.50.0 --depth 1 --recursive \
    https://github.com/IntelRealSense/librealsense.git
tar --exclude='.git' -czf librealsense-2.50.0.tar.gz librealsense/
scp librealsense-2.50.0.tar.gz unitree@192.168.123.164:/home/unitree/

# On the robot:
ssh unitree@192.168.123.164
cd /home/unitree && tar -xzf librealsense-2.50.0.tar.gz
cd librealsense && mkdir -p build && cd build
cmake .. \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_GRAPHICAL_EXAMPLES=OFF \
  -DBUILD_TOOLS=OFF \
  -DBUILD_UNIT_TESTS=OFF \
  -DBUILD_WITH_TM2=OFF \
  -DBUILD_PYTHON_BINDINGS=ON \
  -DPYTHON_EXECUTABLE=/home/unitree/miniconda3/envs/unitree_deploy/bin/python \
  -DCMAKE_BUILD_TYPE=Release \
  -DCHECK_FOR_UPDATES=OFF
make -j4        # ~10 min on the robot's aarch64 CPU
```

Result:
```
/home/unitree/librealsense/build/wrappers/python/pyrealsense2.cpython-310-aarch64-linux-gnu.so
/home/unitree/librealsense/build/librealsense2.so.2.50.0
```

No sudo, no apt, no system-wide install. The bindings link against
the local build's `librealsense2.so.2.50.0`, not the
`ros-noetic-librealsense2` apt copy, so the `LD_LIBRARY_PATH` export
above is mandatory.

## Coordinate conventions

**Camera optical frame** (pyrealsense2 default):
- `+X` right, `+Y` down, `+Z` forward

**Body frame** (matches the control-mode packages):
- `+X` forward, `+Y` left, `+Z` up
- Origin at the camera mount point (not the torso root)

The camera is rotated by a downward pitch `t` around the body X axis.
With `cam = (cx, cy, cz)`:

```
body_x =  cz·cos(t) + cy·sin(t)   # forward
body_y = -cx                       # left (camera-right is body-right = -left)
body_z = -cy·cos(t) + cz·sin(t)   # up (camera-down at zero tilt is body-down)
```

To compose with torso-root coordinates you still need the head
pan/tilt joint angles and the URDF. That's the caller's
problem; `DepthCameraSight` stops at the camera mount frame.

## Calibrating the downward tilt angle

`CAMERA_TILT_DEG_DEFAULT` in `depth_camera_sight.py` is specific to one
physical robot — on the development G1 EDU it is currently **51.29°**,
calibrated 2026-04-12 via `_calibrate_tilt.py`. On any other robot it
will differ and must be re-measured.

### How the calibration script works

`_calibrate_tilt.py` uses the **floor plane as a known reference**.
Every valid depth pixel that's on the floor must, after back-projection
and rotation into body frame, lie at `body_z = -H_cam` (camera mount
height below the camera origin). With ~20 k floor pixels that's a
massively overdetermined constraint. The script does two fits:

1. **Constrained** (`H` fixed to the user-measured camera height):
   per-pixel least-squares on `cy·cos(t) + cz·sin(t) = H` with an
   iterative inner-70% outlier trim.
2. **Unconstrained** (both `t` and `H` free): fits a plane in the
   camera frame via SVD of the centred point cloud, then decomposes
   the plane normal into `(t, H)`. This is the *authoritative* one —
   it doesn't depend on the user getting the tape-measure right.

On the dev robot the unconstrained fit came out at **51.29° tilt,
1.242 m mount height, 0.83 cm RMS** on 18 k inliers. The user's tape
measurement of 4' 0.5" (1.232 m) was within an inch of the SVD's
1.242 m; the SVD is more trustworthy because it uses ~18 000 pixels
instead of one tape endpoint.

### How to re-calibrate

```bash
# Place a known target on the floor in front of the robot (bottle,
# taped square, anything visible in RGB).  Measure the camera mount
# height to the floor.  Then:

source robotics-connect/depth_camera_sight/setup_env.sh
python robotics-connect/depth_camera_sight/_calibrate_tilt.py \
    --camera-height-m 1.242 \
    --bottle-forward-m 0.9144

# Take the "PLANE-SVD FIT (unconstrained)" tilt_deg value and update
# CAMERA_TILT_DEG_DEFAULT in depth_camera_sight.py.
```

Run it 3–5 times back-to-back and take the mean — inter-run variance is
typically well under 0.2°, but the robot's balance mode causes ~1–2°
drift on the timescale of minutes. If you see more drift than that,
something is physically different (bumped head, neck joint not at
nominal) and you should recalibrate.

### Long-term fix: read the neck joint at runtime

The current single-constant approach doesn't account for head pitch
drift. Once the G1 URDF lands, the right answer is to subscribe
to `rt/lowstate`, read the neck pitch joint, and compute the runtime
tilt as `CAMERA_TILT_DEG_NOMINAL + neck_pitch_rad`. That's tracked as
hand-eye follow-up work, not part of this module.

## Testing without hardware

```python
from depth_camera_sight import MockDepthCameraSight

cam = MockDepthCameraSight(object_xy_m=(0.4, 0.0))
frame = cam.latest()
assert frame.depth_m.shape == (480, 640)
body = cam.pixel_to_body_xyz(320, 300)  # expect ~[0.something forward, 0 left, -0.74 up]
```

`MockDepthCameraSight` synthesises a plane at `table_height_m` below
the camera with an optional cylindrical "object" for ACT-lite synthetic
demo generation.

## Gotchas already learned

1. **Do not `mv`/`rm` anything under `/home/unitree/`** before checking
   `pip list --editable`. The robot-side clones are load-bearing
   editable pip installs — this has bitten us before. This applies
   especially to `/home/unitree/unitree_sdk2_python` which this module
   imports.
2. **Use `scp`, not `rsync`**, to deploy to the robot. Verify with a
   grep sentinel on the remote copy.
3. **Phantom ROS topics**: this module deliberately does not use ROS.
   It talks to librealsense directly and to the Unitree video hub via
   DDS. No `roscore` needed.
4. **pip-installed pyrealsense2 does not work on this robot.** The
   wheel on PyPI needs GLIBC 2.32; the robot has 2.31. Build from
   source (section above) instead.
5. **`ChannelFactoryInitialize` is a global.** `DepthCameraSight`
   guards against double-initialisation when another control mode has
   already called it, using a module-level flag. If
   you see DDS errors, confirm no other component is also calling it
   from a different process with a different domain ID.
6. **Depth max 65.535 m is garbage**, not 65 m of real distance. Mask
   out before use. Sensor out-of-range saturates there instead of
   returning 0 (which is the usual "no measurement" value).
7. **The RGB and depth resolutions differ** (1920×1080 vs 640×480).
   That is by design — see the Architecture section. If you try to
   index into `rgb` with depth pixels, you will get garbage. Use the
   right intrinsics dict for the right stream.
