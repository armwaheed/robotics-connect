# SAFETY — operating a real humanoid under low-level / learned control

This document is the **non-negotiable** safety layer for running motor-level control on a
physical humanoid through robotics-connect: velocity-walk, `rt/arm_sdk` overlays, and
especially **whole-body RL-policy deploy** (`rt/lowcmd` with the vendor balance controller
released). It is written from a real incident — read it before any of the on-hardware
control skills ([deploy-policy](skills/deploy-policy/SKILL.md), [locomotion](unitree/g1/locomotion/README.md),
[controller](unitree/g1/controller/README.md)).

It is humanoid-general; G1 specifics are called out.

---

## 0. The cardinal rule

> **NEVER hard-kill (`kill -9` / `SIGKILL`) a process that is sending low-level motor
> commands to a real robot. A hard kill is the OPPOSITE of a stop.**

When the publisher of `rt/lowcmd` (or any motor-command topic) dies without sending a final
safe command, the robot's motor controller keeps acting on the **last command it received** —
a *position target held at the configured stiffness*. The DDS layer makes this worse: the
writer's **last sample persists** to the motor subscriber (and a buffered sample can flush as the
writer is torn down), so the stale high-gain target keeps being applied with nothing left to
update or damp it. If those were policy targets at
transfer/sim gains (on the G1, `kp` up to **150–200**), the joints drive **hard** toward that
posture with no further updates and no rescue. On a downed or unsupported robot that is a
violent runaway.

**What this looked like (2026-06-12, G1 whole-body bed-reach deploy):** a whole-body policy
run on a gantry didn't transfer; the operator pressed the controller **abort, which correctly
damped it**. The deploy process was still alive, so it was `kill -9`'d "to stop the commands."
That hard-kill left the last high-gain reach command latched on the motors with no publisher to
update or damp it — the robot **spin-kicked on the floor and broke a window.** The abort was the
right stop; the hard-kill undid it.

A `SIGKILL` cannot be caught, so a process cannot damp on the way out. That is *why* you never
use it as a stop — and why every deploy process must install a catchable-signal damp handler
(§2, [`lib/safe_stop.py`](lib/safe_stop.py)).

---

## 1. The stop hierarchy (most authoritative first)

Always have the higher tiers physically in reach before any motion.

1. **Hardware e-stop / power cut** — the only *guaranteed* stop. Cuts motor power regardless of
   software state. Keep a hand on it / the battery for every run.
2. **Controller firmware abort / damp** — the handheld's damp (G1: `L2`+`B` / the
   [`G1Remote`](unitree/g1/controller/README.md) any-button latch wired into the control loop).
   Processed by firmware, works even if your process hangs; sets the joints **compliant** (`kp→0`).
3. **Clean software damp** — your control loop commands `kp=0, kd≈small, tau=0` on **all** joints
   for ~1 s and/or re-engages the vendor controller, *then* the process exits. This is the only
   software "stop," and it must be the exit path of every code path (normal, exception, signal).

**A process kill is NOT on this list.** If a process must be terminated, **damp first, confirm
the robot is compliant, then terminate** — and never with `-9`.

---

## 2. Signal-safe deploy (required for any `rt/lowcmd` / `rt/arm_sdk` process)

Every process that commands motors MUST guarantee a damp on **every** exit — normal return,
exception, `SIGINT` (Ctrl-C), and `SIGTERM` (`kill` without `-9`). Use
[`lib/safe_stop.py`](lib/safe_stop.py):

```python
from lib.safe_stop import SafeStop

def damp():               # command compliant on ALL joints (kp=0, kd≈3, tau=0), ~1 s
    g1.publish_damp(seconds=1.0)

with SafeStop(damp):      # damps on return, exception, SIGINT, SIGTERM
    run_control_loop()    # ... your 50 Hz loop, polling the controller abort each tick
```

`SafeStop` cannot protect against `SIGKILL` (uncatchable) or a power loss — those are exactly
why tiers 1–2 above exist. It removes the *catchable* failure modes so the only ways out leave
the robot compliant.

Also keep a **standalone panic-damp** script you can run from a second shell
([`lib/safe_stop.py --panic` pattern]): it opens its own publisher and floods `kp=0` damp to all
joints. Use it to make a robot inert *without approaching it*.

---

## 3. The de-risk ladder — deploying a whole-body policy onto real legs

A first sim→real transfer of a whole-body balance policy **will fall if anything is off** (joint
order, obs scaling, the un-observable base-velocity term, the takeover-pose gap). Failures are
**fast** (sub-second, faster than a catch reflex), not gradual. So stage it; each rung gates the
next:

| Rung | What runs | Fall risk | Gate to pass |
|---|---|---|---|
| **0 — offline** | Build the live obs, run the policy, **print** obs+actions. No commands. | none | obs sane (gravity ≈ `[0,0,-1]`), **joint-order verified** (predicted pose offsets land on the named joints), actions finite/bounded |
| **1 — partial, fall-safe** | Policy runs, but apply **only a subset that can't drop the robot** (e.g. arms via `rt/arm_sdk` while the legs stay on the **vendor balance controller**). Motion-blended, rate-limited, clamped, abort-armed. | none (legs held by vendor) | smooth, bounded, abortable; IMU stays level (legs unaffected) |
| **2 — full whole-body** | All joints via `rt/lowcmd`, **vendor balance released**, motion-blended handover. | **HIGH — can fall** | **robot SUPPORTED** (gantry), abort+e-stop in reach, clear area |

**Rung 2 requires physical support.** Use a **gantry/hoist with a few cm of slack** — the robot
bears its own weight while balancing and the hoist catches a fall in a few centimetres. "Spot it
by hand" is not adequate for a sub-second leg failure. Graduated is best: start the hoist
weight-bearing, slack off as the policy proves it holds. (Note: a vision-less policy reaches a
*target coordinate*, not a perceived surface — you do **not** need the task furniture in front of
it for a balance test, and clear space makes a fall cleaner.)

---

## 4. Handover & gains (rung 2 specifics)

- **Release the vendor controller cleanly and verify it.** Loop `MotionSwitcher.CheckMode()` /
  `ReleaseMode()` past transient RPC misses; if you **cannot confirm** the mode is released,
  **abort without taking over** — never let `rt/lowcmd` fight the vendor controller.
- **Match the sim PD gains** the policy trained under (G1 bed-reach: hip 100/2, knee 150/4,
  ankle 40/2, waist 200/5, arms 40/10). Wrong gains = no transfer.
- **Motion-blend the takeover.** The vendor `BalanceStand` pose is more crouched than the policy
  neutral (G1: knee ≈ 0.52 vs 0.30 rad) — an out-of-distribution start. Hold the captured pose,
  then ramp the target from it to the policy output over ~2–3 s so nothing snaps.
- **Abort / end → low-level damp** (`kp=0, kd≈3`), not a kill (§0). Under a gantry the hoist
  then holds the robot.

---

## 5. After an incident — make it safe **without approaching**

1. **Do not approach a robot that just ran away.** Cut power first if you can.
2. From a safe distance, make it inert in software if it's still commanded: run the **panic-damp**
   (§2) — a `kp=0` flood can only make it compliant, it cannot drive a posture.
3. **Verify inert with read-only telemetry before approaching:** subscribe to `rt/lowstate` and
   confirm, over a few seconds, **max |joint velocity| ≈ 0**, **max |torque| ≈ 0** (passive, not
   driven), the **IMU frozen**, and that **no process is publishing** the command topic. Only then
   approach — ideally from behind / away from the limbs' swing arc; with motors limp the joints are
   back-driveable, not powered.
4. Power off (pull the battery) before re-rigging.

---

## 6. Pre-run checklist (every on-hardware control run)

- [ ] Hardware e-stop / battery in reach; operator's hand on it.
- [ ] Controller abort **armed** and wired into the loop ([`G1Remote`](unitree/g1/controller/README.md)).
- [ ] Process uses [`SafeStop`](lib/safe_stop.py) (damps on return/exception/SIGINT/SIGTERM).
- [ ] Rung 2 only: robot on a **gantry** with slack; clear area; vendor-release verified.
- [ ] You know the **panic-damp** command for a second shell.
- [ ] Verify by **what you see**, not telemetry — but use telemetry to confirm *inert* before approaching.

> Premises are worth challenging everywhere in this project — **except physical safety.** There,
> default to the most conservative stop and never improvise a "just kill it" shortcut mid-motion.
