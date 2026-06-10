# robotics-connect — on-robot verification

Live bring-up plus sensor/end-effector verification of the `unitree/g1` stack on
a real **Unitree G1 EDU** (aarch64 / Tegra, JetPack L4T), **2026-06-03**.

These results are the ground truth for what the sanitized stack actually does on
hardware — preserved here so they never have to be re-derived.

## Scoreboard

| Test | Result | Notes |
|---|---|---|
| Install (`install/install.sh`) | ✅ | Deployed to `/home/unitree/robotics-connect`, `robotics-connect` conda env cloned from factory `unitree_deploy`, profile hook installed. No collision with existing repos. |
| Depth camera (RealSense D435i) | ✅ | 640×480, ~96% valid pixels, 0.40–5.77 m. |
| RGB camera (Unitree VideoClient/DDS) | ✅ | 1920×1080. |
| LiDAR (Livox MID-360) | ✅ | 62–73k pts/scan, floor −1.24 m; **180° mount-roll correction confirmed**. |
| Arm forward kinematics (`arm_fk`) | ✅ | `SELFTEST_OK` + 9/9 regression, **1005 Hz** on the Jetson. |
| Brainco hand — digits | ✅ | All 6 motors/hand incl. lateral thumb (`thumb_aux`). |
| Brainco hand — touch | ✅ | **10/10** fingertips, clean single press events. |
| Brainco hand — proximity | ✅ | **10/10** fingertips; decode corrected and baked into the bridge. |

**Not yet covered (next run):** walking, torso/waist, and arm motion control
(as fresh clean-room Unitree-SDK diagnostics), microphone **and speaker**, a
`--verify` installer flag, and USB-port auto-detect for the hands.

## Sensor captures

- **RGB + colorized depth** — [`unitree/g1/depth_camera_sight/`](unitree/g1/depth_camera_sight/README.md#sample-capture-on-robot-2026-06-03)
- **LiDAR near field (bottle + ball, x-y and x-z) + mid/far room walls** — [`unitree/g1/lidar_sight/`](unitree/g1/lidar_sight/README.md#sample-capture-on-robot-2026-06-03)

## Hand / USB mappings

Full tables (USB ports, digit motors, touch sensors, proximity sensors) with
measured values live in
[`unitree/g1/brainco_touch/README.md`](unitree/g1/brainco_touch/README.md#on-robot-verification--mappings-2026-06-03).
Key gotchas captured there:

- **USB-port trap** — both hands are channels of a single FTDI quad chip, so
  VID/PID **and serial are identical** across all four `ttyUSB*` ports. A hand
  can only be identified by Modbus probe (left `0x7e`, right `0x7f`), and port
  assignment drifts across robots/reboots — always probe, never hard-assume.
- **Proximity decode** — the per-finger value is `touch_raw[16 + 2·i]` (u16,
  ~0 idle → ~65535 near), now exposed directly as `left_proximity` /
  `right_proximity`.

## Effector ground truth — DOF, morphology, factory gains (2026-06-09)

Read off the **live G1 EDU** during a `discover-robot` pass, captured into the robot descriptor
[`skills/discover-robot/descriptors/unitree_g1_edu.json`](skills/discover-robot/descriptors/unitree_g1_edu.json).
This is the real-to-sim ground truth that the Isaac-staging skills reconcile the 29-DOF sim asset against.

| Fact | Value | Source on the robot |
|---|---|---|
| Body DOF | **23** (12 legs + 1 waist + 10 arms) | 23 revolute joints in `g1_description/g1_23dof_mode_10.urdf` |
| Waist | **yaw only** (no roll/pitch) | URDF + `unitree_sdk2_python` G1JointIndex enum |
| Arms | **5/side** — shoulder p/r/y, elbow, **wrist_roll only** (no wrist pitch/yaw) | URDF (`g1_arm5` SDK example) |
| Absent vs. 29-DOF | `waist_roll`, `waist_pitch`, L/R `wrist_pitch`, L/R `wrist_yaw` | SDK enum marks all six **"INVALID for g1 23dof"** |
| Factory PD gains | legs Kp 60/60/60/100/40/40, waist Kp 60/40/40, arms Kp 40 (Kd legs 1/1/1/2/1/1, arms 1) | `unitree_sdk2_python/example/g1/low_level/g1_low_level_example.py` |
| Head camera tilt | **51.29° down** (floor-plane SVD) | `CAMERA_TILT_DEG_DEFAULT` on-disk |
| Hands | **Brainco** 5-finger (6 motors), over the FTDI **FT4232H** quad | USB (`lsusb`) + `brainco_touch` |
| Sensors on USB | Intel RealSense D435i (`8086:0b3a`) | USB |

> **Real vs. sim.** The physical EDU is **23-DOF with Brainco hands**; the Isaac G1 asset is **29-DOF
> with Inspire hands**. The descriptor records both and locks the 6 absent sim joints so a trained policy
> is transfer-valid. The full **29-DOF** G1 (the common research config) is the clean case where nothing
> is locked — see [`unitree_g1_29dof.json`](skills/discover-robot/descriptors/unitree_g1_29dof.json). The
> 23-DOF EDU is one (reduced) variant, **not** the paradigm.

## Environment

![G1 EDU test environment](unitree/g1/images/environment_overview.jpg)

*G1 EDU on a gantry harness facing the artist's-palette table (water bottle +
blue balance ball) in a home office. The glass french doors behind explain the
LiDAR mid/far through-glass returns.*

- Robot: `unitree@192.168.123.164` — G1 EDU, aarch64, `5.10.x-tegra`.
- Factory `unitree_deploy` conda env present; `robotics-connect` env cloned from it.
- Depth via librealsense (`/home/unitree/librealsense/build`); RGB via Unitree
  VideoClient over DDS; LiDAR via DDS topic `rt/utlidar/cloud_livox_mid360`.
