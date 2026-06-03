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

## Environment

![G1 EDU test environment](unitree/g1/images/environment_overview.jpg)

*G1 EDU on a gantry harness facing the artist's-palette table (water bottle +
blue balance ball) in a home office. The glass french doors behind explain the
LiDAR mid/far through-glass returns.*

- Robot: `unitree@192.168.123.164` — G1 EDU, aarch64, `5.10.x-tegra`.
- Factory `unitree_deploy` conda env present; `robotics-connect` env cloned from it.
- Depth via librealsense (`/home/unitree/librealsense/build`); RGB via Unitree
  VideoClient over DDS; LiDAR via DDS topic `rt/utlidar/cloud_livox_mid360`.
