# Unitree G1 — Locomotion (walk under closed-loop control)

Drive the G1 to a goal pose: command a body-frame velocity through Unitree's
balance controller and steer on the robot's **measured** odometry. Built for
autonomous tasks like the bed-making demo ([armwaheed/robots#3](https://github.com/armwaheed/robots/issues/3)),
where the robot walks itself to the bed before reaching. Tracking issue:
[robotics-connect#11](https://github.com/armwaheed/robotics-connect/issues/11).

The capability is split into a **robot-agnostic** layer and this **G1 binding**:

| File | Role |
|---|---|
| [`../../../lib/locomotion.py`](../../../lib/locomotion.py) | abstract `LocomotionController` + closed-loop `walk_to` / `turn_to` / `walk_forward`; `SimLocomotion` for off-robot tests |
| [`../../../lib/navigation.py`](../../../lib/navigation.py) | sensor-agnostic `Navigator`: cloud → inflated occupancy grid → A* waypoints |
| [`g1_locomotion.py`](g1_locomotion.py) | `G1Locomotion` — the G1 over `LocoClient` + `rt/odommodestate` |

---

## TL;DR

```python
from g1_locomotion import G1Locomotion

loco = G1Locomotion(iface="eth0")
loco.connect()                 # LocoClient + rt/odommodestate
# Robot must ALREADY be standing (stand it with the controller, or loco.squat_to_stand()
# if it is crouched). Walking is just velocity from a balanced stand — no stand() needed.
loco.walk_forward(2.0)         # 2 m ahead, closed-loop on measured odometry
loco.stop()
```

Any humanoid that implements `set_velocity`, `pose`, and `stop` inherits the
same `walk_to` / `turn_to` / `walk_forward` helpers — the G1 is one binding.

---

## 1. Walk — Unitree `LocoClient` (the balance controller)

`G1Locomotion.set_velocity(vx, vy, vyaw)` calls `LocoClient.Move` — the
manufacturer's high-level velocity interface, so **balance is the controller's
job**. No reinforcement-learning policy is pushed onto the legs. Lifecycle:
`squat_to_stand()` → `Squat2StandUp` (lift from a squat), `stand(mode)` →
`BalanceStand(mode)` (set balance mode), `damp()` → `Damp`, `stop()` → `StopMove`.

Body frame is `+x` forward, `+y` left, `+yaw` counter-clockwise (REP-103).

## 1a. Hardware-verified gotchas (G1 EDU, current SDK)

These cost real debugging on hardware — they are baked into the binding, but know them:

- **`BalanceStand` needs a `balance_mode` arg** on the current SDK (it is `SetBalanceMode`
  underneath: `0` = static, `1` = continuous gait). Calling it with no arg raises `TypeError`.
  It only sets the mode — it does **not** lift the robot from a squat. The robot must already be
  standing; **walking is just `set_velocity` from a stand** (no `stand()` call needed on the walk
  path).
- **Use `continous_move=True`.** `LocoClient.Move(vx,vy,vyaw)` defaults to a **1 s velocity
  pulse**; re-issuing it every 0.1 s in the closed loop makes the gait **re-ramp from rest each
  tick** → a ~0.03 m/s shuffle that also trips the stall guard. `set_velocity` passes
  `continous_move=True` for a sustained gait (measured ~0.2–0.35 m/s). This removes the 1 s
  dead-man, so **every exit must `stop()`** — wrap the walk in a `finally` (see
  [SAFETY.md](../../../SAFETY.md)).
- **The stall guard is a speed check, not an obstacle sensor.** `is_blocked` flags "commanded but
  measured speed < fraction × commanded for a grace window." The G1's low-speed stepping gait dips
  toward zero velocity *between steps*, so a tight threshold false-trips on open ground — calibrated
  loose (`STALL_SPEED_FRACTION=0.10`, `STALL_GRACE_S=4.0`). Don't read a "stalled" as "there's a
  wall"; read it as "not achieving commanded speed."

## 2. Localize — measured, not commanded

Steering reads the robot's **measured** planar pose from `rt/odommodestate`
(`SportModeState_`: `position[3]`, `velocity[3]`, `yaw_speed`) — verified
publishing live on the EDU. This is closed-loop on what the feet *did*, which is
strictly better than integrating the velocity you *asked for*. `is_blocked()`
compares commanded vs. measured speed to detect a stall.

> Right-sized for short approaches (a few metres). Leg odometry still drifts over
> long distances — see the upgrade path below.

## 3. Plan a path — `Navigator` (optional)

For getting *around* obstacles, `lib/navigation.py` turns an `(N, 3)` LiDAR cloud
into an inflated 2-D occupancy grid, plans A* waypoints to the goal, and drives
any `LocomotionController` along them (facing each leg, re-planning on a stall).
The same numpy runs on a simulator's cloud or a real LiDAR cloud.

## Production upgrade (drift-free, cross-room) — no consumer changes

The closed-loop API is unchanged when the backend is swapped for the heavy path:

- **Localization:** LiDAR-inertial odometry on the MID-360 —
  [Point-LIO for Unitree LiDAR](https://github.com/unitreerobotics/point_lio_unilidar),
  or [FAST-LIO localization for the G1](https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID)
  (upside-down mount + prebuilt map → no long-term drift). Feed its pose into
  `G1Locomotion.pose()` and every helper keeps working.
- **Planning at scale:** [Nav2](https://docs.nav2.org/concepts/index.html) costmaps
  ([G1/Go2 config thread](https://github.com/ros-navigation/navigation2/issues/5512)),
  or Unitree's built-in [SLAM & Navigation service](https://support.unitree.com/home/en/developer/SLAM%20and%20Navigation_service).

## Safety

`set_velocity`, `stand`, and the `walk_*` helpers **move the legs**. The caller
owns the physical preconditions: a clear area, an operator on the e-stop, and
adequate battery. The CLI's `--forward` requires a typed confirmation; the
default run only streams the measured pose (no motion). Read the repo-wide
[**SAFETY.md**](../../../SAFETY.md) — the stop hierarchy and the rule that you
**never `kill -9`** a process that is commanding the legs.

```bash
python g1_locomotion.py --iface eth0            # stream measured pose, no motion
python g1_locomotion.py --forward 1.0           # DANGER: 1 m supervised walk
```

## What it feeds `discover-robot`

| Descriptor field | Value |
|---|---|
| `locomotion.api` | Unitree `LocoClient.Move(vx, vy, vyaw)` (balance controller) |
| `locomotion.odometry` | measured, `rt/odommodestate` (`SportModeState_`) |
| `locomotion.frame` | body `+x` fwd / `+y` left; planar pose in the odom frame |

## Sources

`LocoClient` API: `unitree_sdk2py/g1/loco/g1_loco_client.py`. Odometry topic:
`rt/odommodestate` (`unitree_go::msg::dds_::SportModeState_`), confirmed live on
the EDU. Upgrade references linked above.
