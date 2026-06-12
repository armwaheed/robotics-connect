# robotics-connect

**Agent skills + verified-on-hardware control stacks that let an AI agent discover, *operate*, and
real-to-sim-stage a humanoid robot — from plugged-in on the bench, to driving it through a real task
with a human in the loop, to a sim-to-real-valid Isaac Lab RL job on a DGX Spark.**

robotics-connect is a collection of [Claude Code **Agent Skills**](skills/README.md) plus the
verified-on-hardware control stacks they wrap. It has **two pillars**, both driven by the same robot
descriptor:

- **Operate the real robot.** Hardware-verified capability stacks an agent uses to actually run a task:
  **perceive** a scene (depth + LiDAR, body-frame geometry), **walk and navigate** to it, **speak to and
  hear from a human partner** over **Device Connect**, grip with the **hands**, and **abort from the
  controller** — the layers behind a real bed-making G1 that asks a person for help when it gets stuck.
- **Real-to-sim staging.** **Discover and characterize** the robot's real sensor + effector envelope
  into a machine-readable descriptor, **stage a sensor-calibrated, free-base Isaac Lab RL job** from it,
  and **deploy the trained policy** back out into a control loop.

The same flow generalizes to a new humanoid by re-running the skills.

## The real-to-sim thesis

Isaac Sim does **not** fully expose a real robot's sensor + effector envelope. The sensors are tilted,
range-limited, and occluded by the robot's own body in ways the sim won't tell you; the real degrees of
freedom, gains, and hand morphology differ from whatever asset you downloaded. robotics-connect
**characterizes those on the hardware**, and that characterization is **calibration data an agent builds
the sim from** — so a sim-trained policy/detector transfers, and the RL training cycles are spent on a
sim that already matches the robot. robotics-connect is the **real-to-sim** half that feeds sim-to-real.

The artifact at the center of that loop is the **[robot descriptor](skills/discover-robot/schema/robot_descriptor.schema.json)**:
a single machine-readable card describing the *real* robot — its actual DOF + joint order, PD gains,
sensor mounts/tilts/blind-spots, hand morphology, and **how it reconciles with the sim asset** (which
sim DOFs the real robot lacks). Every Isaac-staging skill consumes it.

## Layout

```
robotics-connect/
├── .claude-plugin/              # the plugin + marketplace manifests (install as a Claude Code plugin)
├── skills/                      # robot-AGNOSTIC skills (discovery, perception, Isaac staging, setup)
│   ├── discover-robot/          #   keystone: emit the real-to-sim robot descriptor
│   ├── perceive-surfaces/       #   real-robot surface/object perception (LiDAR-first) → body frame
│   ├── stage-isaac-sensors/     #   sim sensors with the real blind spots
│   ├── stage-isaac-freebase/    #   free-base USD + deploy gains + DOF reconciliation
│   ├── stage-isaac-rl-env/      #   the whole-body reach RL env template
│   ├── deploy-policy/           #   run the trained policy out of the RL env
│   ├── setup-dgx-spark/         #   Isaac Sim + Isaac Lab on a DGX Spark (GB10, aarch64)
│   └── bootstrap-device-connect-env/  # two-env bridge for the Device Connect sidecar (py3.11)
├── unitree/                     # manufacturer
│   └── g1/                      # product — Unitree G1 EDU (robot-SCOPED capability skills + code)
│       ├── depth_camera_sight/  #   head RealSense depth + RGB (each dir has SKILL.md + README + code)
│       ├── lidar_sight/         #   crown Livox MID-360 perception
│       ├── locomotion/          #   walk / navigate to a goal (LocoClient + measured odometry + A*)
│       ├── controller/          #   handheld-remote read → any-button routine abort
│       ├── voice/               #   speak + listen (TTS out, mic→ASR, grounded reply)
│       ├── device_connect/      #   the robot as a Device Connect agent (asks a human for help)
│       ├── arm_fk/              #   pure-numpy URDF forward kinematics
│       ├── brainco_touch/       #   Brainco 5-finger hands (digits, touch, proximity)
│       ├── vision_sidecar/      #   containerized GPU (DINOv2) inference sidecar
│       ├── install/             #   on-robot deploy / uninstall / offline bundle
│       └── connect/             #   host ↔ robot networking (configure_*.sh + CycloneDDS)
├── human_agent/                 # a HUMAN as a Device Connect device (headset + ASR; the robot asks it for help)
├── lib/                         # shared modules (the Device Connect sidecar boilerplate, one copy)
└── assets/media/                # validation media — the "what good looks like" reference standards
```

Robot-scoped skills stay co-located with the verified-on-hardware code (keeping the manufacturer/product
layout); robot-agnostic skills are flat under `skills/`. Both are registered in
[`.claude-plugin/plugin.json`](.claude-plugin/plugin.json), so an agent discovers them uniformly.
Start at the **[skills catalog](skills/README.md)**.

## Verified on hardware

The `unitree/g1` stack has been brought up and verified **live on a real Unitree G1 EDU** — both pillars.
*Perception/effectors:* install, depth/RGB/LiDAR, arm forward kinematics, the Brainco hands. *Operate:*
the robot **speaks** (chest-speaker TTS), runs the **human-in-the-loop help loop over Device Connect**
(it asks a person and acts on the spoken reply), reports **measured odometry** (`rt/odommodestate`) for
closed-loop walking, and **aborts from the handheld controller** (any-button latch, verified button-by-
button). The robot's **23-DOF body**, **yaw-only waist**, **roll-only wrists**, **factory PD gains**, and
**sensor envelope** were read off the live robot into
[`skills/discover-robot/descriptors/unitree_g1_edu.json`](skills/discover-robot/descriptors/unitree_g1_edu.json).
See **[`VERIFICATION.md`](VERIFICATION.md)** for the full scoreboard, the on-hardware ground truth, and
the hand/USB mapping tables.

## Adding a robot

Run [`discover-robot`](skills/discover-robot/SKILL.md) to produce a new descriptor; the Isaac-staging
skills consume it unchanged. Robot-scoped capability code goes in a new `<manufacturer>/<product>/`
directory, module-per-capability like `unitree/g1/`. See
[`skills/discover-robot/references/onboarding-new-humanoid.md`](skills/discover-robot/references/onboarding-new-humanoid.md)
for the generalization seams (worked for the Unitree H2 Plus and Boston Dynamics Atlas).
