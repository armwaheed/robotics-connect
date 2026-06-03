# robotics-connect

A toolkit of robot control stacks for use with **Arm AI Fabric**. Code is
organised by robot **manufacturer** and **product**, so each robot's control
and perception stack is self-contained and independently deployable.

## Layout

```
robotics-connect/
└── unitree/                      # manufacturer
    └── g1/                       # product — Unitree G1 EDU humanoid
        ├── arm_fk/               # pure-numpy URDF forward kinematics for the arms
        ├── brainco_touch/        # Brainco Revo2 Touch hand bridge (Modbus → TCP JSON)
        ├── depth_camera_sight/   # Intel RealSense depth-camera perception
        ├── lidar_sight/          # LiDAR perception + scene mapping
        ├── vision_sidecar/       # containerised GPU (DINOv2) inference sidecar
        ├── install/              # on-robot deploy / uninstall / offline bundle
        ├── configure_*.sh        # host ↔ robot network configuration
        └── cyclonedds.xml        # DDS unicast config
```

See [`unitree/g1/README.md`](unitree/g1/README.md) for the Unitree G1 EDU
control stack.

## Adding a robot

Create a `<manufacturer>/<product>/` directory and place that robot's
control stack inside it, following the same self-contained, module-per-
capability convention as `unitree/g1/`.
