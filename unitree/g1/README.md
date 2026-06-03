# Unitree G1 EDU — control stack

The **perception, hand-control, and deployment** stack for the Unitree G1 EDU
humanoid, part of the [`robotics-connect`](../../README.md) toolkit. Every
capability is a self-contained module with its own README; this page is the
walkthrough — what's here, how to install/uninstall it, and where to go to
try each piece on the robot.

## What's included

| Capability | Module | Where to start |
|---|---|---|
| **Brainco hands** — open/close each digit, read touch + proximity | [`brainco_touch/`](brainco_touch/) | [`brainco_touch/README.md`](brainco_touch/README.md) |
| **Depth camera** (RealSense D435i) + IR | [`depth_camera_sight/`](depth_camera_sight/) | [`depth_camera_sight/README.md`](depth_camera_sight/README.md) |
| **RGB camera** (Unitree video over DDS) | [`depth_camera_sight/`](depth_camera_sight/) | same service |
| **LiDAR** (Livox MID-360) — mid-field scene perception | [`lidar_sight/`](lidar_sight/) | [`lidar_sight/README.md`](lidar_sight/README.md) |
| **Arm geometry** — forward kinematics (palm/elbow/wrist XYZ) | [`arm_fk/`](arm_fk/) | [`arm_fk/README.md`](arm_fk/README.md) |
| **GPU inference sidecar** (DINOv2 embeddings over local RPC) | [`vision_sidecar/`](vision_sidecar/) | [`vision_sidecar/README.md`](vision_sidecar/README.md) |
| **Deploy / networking** | [`install/`](install/), `configure_*.sh` | [`install/README.md`](install/README.md) |

## What's *not* included

This is a perception + hand-control stack. It intentionally does **not** ship:

- **Locomotion** (walking), **torso/waist motion**, or **arm/joint motion
  control.** `arm_fk` reasons about *where* the arms are but never commands
  them. The motion layers were higher-level feature modules left out of the
  sanitized migration.
- **Microphone / audio capture** — no module exists for it.

If those are needed here later, they should be added as fresh modules built
on the public Unitree SDK, not carried over.

## Install

The installer deploys this package to `/home/unitree/robotics-connect/` on the
robot, creates a dedicated `robotics-connect` conda env (cloned from the
factory `unitree_deploy`), and installs a shell activation hook. It is
idempotent and fully reversible. See [`install/README.md`](install/README.md)
for the full guide, offline-bundle workflow, and what it does / does not touch.

```bash
# On the robot, from the deployed source tree:
bash install/install.sh                 # core stack
WITH_SIDECAR=1 bash install/install.sh  # also install the ~12 GB GPU sidecar

# First-time bring-up of a fresh robot (WiFi + env + install in one shot):
SSID="YourNetwork" WIFI_PASS="..." bash install/first-boot.sh
```

The Brainco hand bridge installs separately (it targets the `g1brainco` env):

```bash
bash brainco_touch/install_brainco_touch.sh [unitree@<host>]
```

## Uninstall

```bash
bash /home/unitree/robotics-connect/install/uninstall.sh
```

Returns the robot to factory state (modulo the conda package cache) and stops
+ removes the `robotics-connect-*` systemd units.

## Trying each capability on the robot

Each module's README has the exact, copy-pasteable commands. In short:

- **Hands** — `bash brainco_touch/install_brainco_touch.sh` starts the bridge
  on `127.0.0.1:9877`; `python brainco_touch/smoke_test.py` confirms touch is
  live. Commanding digits is a `{"cmd":"set","left":[…6…],"right":[…6…]}` JSON
  line to that port (0=open … 1=closed); touch + proximity come back on
  `{"cmd":"get"}`. See [`brainco_touch/README.md`](brainco_touch/README.md).
- **Depth + IR camera** — `python depth_camera_sight/_diag_camera_test.py` is a
  one-shot "is the camera alive" check; `_capture_ir.py` saves IR frames. See
  [`depth_camera_sight/README.md`](depth_camera_sight/README.md).
- **RGB camera** — served by the same `DepthCameraSight` service via the
  Unitree VideoClient (DDS); see the depth-camera README for the RGB path.
- **LiDAR** — `python lidar_sight/_diag_scene.py` prints a live body-frame
  histogram + floor/table search; `_capture_frames.py` renders annotated
  JPEGs. See [`lidar_sight/README.md`](lidar_sight/README.md).
- **Arm FK** — `python arm_fk/arm_fk.py` runs the selftest off-robot (no
  hardware needed); `python arm_fk/test_arm_fk.py` runs the regression suite.

## Connecting to the robot (networking)

The robot's `eth0` lives on its own `192.168.123.0/24` subnet. The top-level
`configure_robot.sh` / `configure_spark.sh` (and their `revert_*` counterparts)
route a control host to that subnet over WiFi, and `cyclonedds.xml` configures
CycloneDDS for unicast. Run order and details are in the per-script headers;
point `CYCLONEDDS_URI` at the deployed copy:

```bash
export CYCLONEDDS_URI=file:///home/unitree/robotics-connect/cyclonedds.xml
```
