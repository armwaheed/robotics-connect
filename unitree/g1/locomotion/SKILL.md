---
name: unitree-g1-locomotion
description: >-
  Walk the Unitree G1 to a goal under closed-loop control — command a body-frame velocity through
  Unitree's high-level LocoClient (the manufacturer balance controller, no RL on the legs) and steer
  on the robot's MEASURED odometry from rt/odommodestate. Provides walk_to(xy), turn_to(yaw),
  walk_forward(distance), and (optional) Navigator path-planning that turns a LiDAR cloud into an
  inflated occupancy grid and A* waypoints around obstacles. Use when a humanoid needs to APPROACH a
  target (e.g. walk to the bed before reaching) or navigate a clear/cluttered space. Robot-agnostic
  core in lib/; the G1 is one binding.
metadata:
  tags: [unitree-g1, locomotion, walking, navigation, odometry, path-planning, mobility, deviceconnect]
---

# Unitree G1 — Locomotion (walk + navigate)

Command a velocity through Unitree's balance controller and close the loop on measured odometry. The
API, the localization choice, the optional planner, the production upgrade path (Point-LIO / FAST-LIO
/ Nav2), and the safety rules are in **[`README.md`](README.md)** — this skill is the agent entry
point. Modules: [`g1_locomotion.py`](g1_locomotion.py), [`../../../lib/locomotion.py`](../../../lib/locomotion.py),
[`../../../lib/navigation.py`](../../../lib/navigation.py).

## When to use

- The robot must **approach** something before acting on it (walk to the bed, to a table, to a person).
- You need a humanoid to **walk to a goal pose** holding a heading, or **turn in place**.
- You need to **route around obstacles** from a LiDAR cloud (the `Navigator`).
- You're filling a robot **descriptor**'s `locomotion` block (velocity API + odometry source + frame).

## How to use

```python
import sys; sys.path.insert(0, "unitree/g1/locomotion")
from g1_locomotion import G1Locomotion

loco = G1Locomotion(iface="eth0"); loco.connect(); loco.stand()
loco.walk_forward(2.0)          # closed-loop on rt/odommodestate
loco.stop()
```

For obstacle-aware navigation, feed a live cloud to the `Navigator`:

```python
sys.path.insert(0, "lib")
from navigation import Navigator
Navigator().navigate(loco, goal_xy=(3.0, 0.0), cloud_source=get_lidar_cloud)
```

Off-robot, swap `G1Locomotion` for `SimLocomotion` (in `lib/locomotion.py`) — the behaviour code and
the closed-loop helpers are identical, so the whole flow is testable in loopback.

## Safety

`set_velocity` / `stand` / `walk_*` **move the legs**. Confirm a clear area, an operator on the
e-stop, and adequate battery before commanding motion. The `--forward` CLI requires a typed
confirmation; the default run only streams the measured pose.
