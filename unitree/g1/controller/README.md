# Unitree G1 — Controller (handheld remote → abort signal)

Read the G1's handheld controller and turn **any button press into a clean abort**
of an autonomous routine. Built so a human supervising a hands-off routine (e.g. the
bed-making approach) can halt it from the controller they're already holding.
Module: [`g1_remote.py`](g1_remote.py).

> ## ⚠️ This is NOT the emergency stop
> A software watcher that reads `rt/lowstate` is only as alive as this process and DDS.
> If those hang — exactly when you'd hit the button — it won't fire. The controller's
> **firmware-level damping** and the robot's **physical power / e-stop** work regardless
> and are the real stop. Keep them in reach. This watcher is a *convenience*: a clean,
> one-button halt of the **software routine** (stop motion, hold balance — it does not
> collapse the robot).

---

## TL;DR — wire an abort into a routine

```python
from g1_remote import G1Remote
remote = G1Remote(iface="eth0", init_dds=False)   # G1Locomotion already inited DDS
remote.connect()
remote.wait_until_armed()                          # arms once buttons are released
loco.set_abort_source(remote.aborted)              # walk_to/turn_to now stop on a press
...
while not remote.aborted() and corners_left:       # also poll between steps
    do_next_step()
```

`set_abort_source` is on the robot-agnostic [`LocomotionController`](../../../lib/locomotion.py):
the closed-loop helpers poll it every tick and return `"aborted"` the instant a button is
pressed. The G1 controller is one abort source; any humanoid with a halt switch can supply
its own.

## Behaviour

- **Any button** trips the abort — `R1 L1 start select R2 L2 F1 F2 A B X Y` + the d-pad.
- **Arms after release.** `aborted()` stays False until the buttons are first seen released
  (so a button held at start-up doesn't trip it), then **latches** on the first press.
- Sticks are read too (`sticks()` → `lx/ly/rx/ry`) but **do not** trip the abort by default —
  they drift, and a spurious abort that stops a walking robot is itself a hazard.

## Read-only dry test (do this before trusting it)

Never trust an unverified safety path. With the robot **in position, no routine running**:

```bash
python g1_remote.py --iface eth0      # READ-ONLY, no motion
# press each button → expect: buttons=[...]  armed=True  ABORTED=True
```

## Frame layout

`LowState_.wireless_remote` is a 40-byte `xRockerBtnDataStruct`: `head[2]`, **button
bitmask uint16 at `[2:4]`**, then floats `lx[4:8]`, `rx[8:12]`, `ry[12:16]`, `L2[16:20]`,
`ly[20:24]`. Verified on the EDU (bitmask `0x0000` at rest). The dedicated
`rt/wirelesscontroller` topic is silent on this firmware, so we parse `rt/lowstate`.

## What it feeds `discover-robot`

| Descriptor field | Value |
|---|---|
| `controller.source` | `rt/lowstate.wireless_remote` (uint16 bitmask @ [2:4] + 4 float axes) |
| `controller.abort` | any-button latch → `LocomotionController.set_abort_source` (stop + hold) |
| `safety.estop` | **firmware damping / physical power — not this watcher** |
