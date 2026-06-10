# Brainco Revo2Touch — real touch sensors

As of 2026-04-14 the Brainco V2 hands on this G1 expose per-finger **normal
force** and **tangential force** over Modbus RTU.  The bridge
(`brainco_bridge.py`) enables the sensors at startup, polls
register 4200, and publishes the values on the TCP JSON protocol.

## Why this was painful

The installed `libbc_stark_sdk.so` (version **0.4.3**) hard-codes every
touch API as "deprecated for current firmware" and returns empty, which
is where the earlier "touch unsupported" conclusion came from.
The SDK ships a newer enum value `STARK_HARDWARE_TYPE_REVO2_TOUCH = 4`
starting in **0.8.1**, and only rejects touch reads when the caller
passes the non-touch `REVO2_BASIC = 3` firmware type — which the
installed ROS2 wrapper `params_v2_double.yaml` does.

Dropping in the 0.8.1 `.so` and passing `fw_type = 4`:

```
modbus_get_device_info(left_slave=0x7e) →
  hardware_type = 4   (STARK_HARDWARE_TYPE_REVO2_TOUCH)
  sku_type      = 2   (left)
  serial_number = BCXTL2334J250000D
  firmware_version = 1.0.14.U
```

confirms the hands are the Touch variant physically.  Both the left-
and right-hand units register 4/5 and 5/5 fingertip channels firing
crisply (idle normal_force ≈ 0, firm press ≳ 2500 raw).

## What the bridge actually does

We bypass `libbc_stark_sdk.so` entirely and talk to the same Modbus
registers via `pyserial`, mirroring what 0.8.1's
`modbus_get_touch_status` does internally.  This keeps `brainco_bridge.py`
dependency-free and avoids swapping out the installed SDK.

## On-robot verification (2026-04-14)

Single-finger raw-register press tests, bridge TCP poll while the user
pressed one fingertip at a time:

| Test           | Target register | Observed     | Verdict |
| -------------- | --------------- | ------------ | ------- |
| LEFT  thumb    | `reg[0]`        | 2500 (sat.)  | ✅      |
| LEFT  index    | `reg[3]`        | 2500 (sat.)  | ✅      |
| LEFT  middle   | `reg[6]`        | 2500 (sat.)  | ✅      |
| LEFT  ring     | `reg[9]`        | 2500 (sat.)  | ✅      |
| LEFT  pinky    | `reg[12]`       | 2500 (sat.)  | ✅      |
| RIGHT thumb    | `reg[0]`        | 2500 (sat.)  | ✅      |
| RIGHT index    | `reg[3]`        | 2500 (sat.)  | ✅      |
| RIGHT middle   | `reg[6]`        | 2500 (sat.)  | ✅      |
| RIGHT ring     | `reg[9]`        | 2500 (sat.)  | ✅      |
| RIGHT pinky    | `reg[12]`       | 2500 (sat.)  | ✅      |
| LEFT  **palm** | any of 0..14    | ~0           | no sensor |

All 10 fingertip channels saturate cleanly at 2500 when pressed firmly
and sit at 0 when idle.  The rubber patches on the palm are grip-only;
there is no touch sensor under them.

### `0xFFFF` "no data" sentinels

The firmware writes `0xFFFF` on unused force fields (especially the
`tangential_direction` register of idle fingers).  The bridge parser
masks these to 0 in `_parse_touch_regs` so downstream callers never see
the spurious ~65535 as if it were a real force.  Proximity registers
(15..24) and the per-finger counters (25..29) can legitimately reach
~65535 and are left alone.

### Baseline idle noise floor

A small persistent nonzero reading sits on some right-hand tangential
registers: `reg[5] ≈ 178`, `reg[14] ≈ 180` with the hand idle and nothing
touching the fingertips.  These are well under `LOAD_NORM = 500`, so
they don't trip `FINGER_CONTACT_THRESHOLD = 0.8` on the exposed `touch`
array.  Flagging for future reference: if `LOAD_NORM` is ever lowered
this will need a software baseline subtraction.

### Enable sequence (once at startup, per hand)

```
Modbus write holding registers  addr=4000  count=5  data=[1,1,1,1,1]   # enable
(wait 300 ms)
Modbus write holding registers  addr=4010  count=5  data=[1,1,1,1,1]   # calibrate idle baseline
```

Failing the enable write falls the bridge back to the old position-lag
"touch" computation so nothing downstream breaks.

### Read loop (every motor-status tick, ~25 Hz)

```
Modbus read input registers  addr=4200  count=30
```

Returns 30 × `uint16`.  Layout (empirically verified 2026-04-14 by
time-aligning single-finger presses to individual positions):

| regs    | meaning                                              |
| ------- | ---------------------------------------------------- |
| `0..14` | 3 values per finger, f0..f4 interleaved              |
|         | `[normal_force, tangential_force, tangential_dir]`   |
| `15..24`| 2 values per finger, f0..f4 interleaved              |
|         | `[self_proximity_lo, self_proximity_hi]` (u32 halves)|
| `25..29`| per-finger status/counter — looks like a frame id, ignore |

Finger order matches the Brainco SDK header: `f0=thumb, f1=index,
f2=middle, f3=ring, f4=pinky`.

### JSON fields emitted by the bridge

- `left_touch` / `right_touch` — 5 floats, `normal_force / 500` clamped to
  `0..1`.  **Backwards compatible** with every caller that already reads
  `state[4]`/`state[5]`.
- `left_touch_force` / `right_touch_force` — 5 raw `uint16` normal forces.
- `left_touch_raw` / `right_touch_raw` — full 30-register dump for
  debugging and future fields (tangential direction, proximity).
- `left_proximity` / `right_proximity` — 5 `uint16` per-finger self-proximity
  values (thumb, index, middle, ring, pinky), decoded from
  `touch_raw[16 + 2·i]` (only the second register of each pair carries the
  signal). ~0 at rest, climbing toward ~65535 as an object nears the
  fingertip. Verified on-robot 2026-06-03 across all 10 fingertips.
- `left_touch_ok` / `right_touch_ok` — `True` if the enable write
  succeeded and the bridge is publishing real sensor data rather than
  the position-lag fallback.

## Temperature — not yet

`libbc_stark_sdk.so` 0.8.1 does not export any temperature read.  There
is a per-motor `states[6]` byte array in the motor-status block that
might carry thermal bits (not yet decoded).  A temperature
requirement is still open as a follow-up; the touch requirement is the
critical half and is what this bring-up delivers.

## Re-running the probe from scratch

The investigation lived in the attached session, but a full redo is
roughly:

```
# Drop the newer Stark SDK shared library on the robot
scp libbc_stark_sdk_0.8.1.so unitree@<g1>:/tmp/

# Probe device info / register 4200 via ctypes
python3 probe_touch_v2.py 4        # fw_type=4 = REVO2_TOUCH

# Confirm the register mapping with a finger-by-finger press log
STEPS=info,enable,calibrate LIVE=1 LIVE_SECS=30 HAND=left \
    python3 probe_touch_v2.py 4
```

Or, once the bridge is running, just:

```
python3 -c "
import json, socket
s = socket.socket(); s.connect(('127.0.0.1', 9877))
s.sendall(b'{\"cmd\":\"get\"}\n')
print(json.loads(s.makefile().readline())['left_touch_force'])
"
```

## On-robot verification & mappings (2026-06-03)

Verified live on a Unitree G1 EDU with Brainco Revo2 **Touch** hands. Every
mapping below was confirmed on real hardware during robotics-connect bring-up.

### USB port mapping (hand connectivity)

Both hands are FTDI USB-serial dongles that, on this robot, enumerate as
channels of a **single FTDI quad chip** — so VID/PID **and serial are
identical across all four ports** and cannot identify a hand. The only
reliable identification is a **Modbus probe** (left hand answers slave
`0x7e`, right hand `0x7f`). **Port assignment can drift across robots and
reboots — always probe, never hard-assume.**

| Port | Device | VID:PID | Serial | Role (this robot) |
|---|---|---|---|---|
| `/dev/ttyUSB0` | FTDI quad ch A | `0403:6011` | `FTB3GNPM` | not a hand (no `0x7e`/`0x7f`) |
| `/dev/ttyUSB1` | FTDI quad ch B | `0403:6011` | `FTB3GNPM` | **Left hand** — Modbus slave `0x7e` |
| `/dev/ttyUSB2` | FTDI quad ch C | `0403:6011` | `FTB3GNPM` | **Right hand** — Modbus slave `0x7f` |
| `/dev/ttyUSB3` | FTDI quad ch D | `0403:6011` | `FTB3GNPM` | not a hand (no `0x7e`/`0x7f`) |

**Auto-detect (2026-06-10):** the bridge now **probes** for the hands by Modbus slave id instead of
trusting a hard-coded port — `detect_hand_ports()` runs by default, and `--detect` prints the mapping:

```bash
python brainco_bridge.py --detect
# probe: /dev/ttyUSB1 answers Modbus 0x7e -> left hand
# probe: /dev/ttyUSB2 answers Modbus 0x7f -> right hand
# {"left": "/dev/ttyUSB1", "right": "/dev/ttyUSB2", "scanned": [...]}
```

So a reboot that reshuffles the FTDI channels no longer breaks the hands. Pass `--port-l`/`--port-r` to
override the probe; baud `460800`. (The legacy `DEFAULT_PORT_*` constants remain only as documentation.)

### Digit (motor) mapping — `set` command, `[6 floats, 0..1]`

| idx | motor | `0.0` | `1.0` |
|---|---|---|---|
| 0 | thumb_curl | open | closed |
| 1 | thumb_aux | slap (thumb abducted from palm) | claw (thumb opposed across palm) |
| 2 | index | open | closed |
| 3 | middle | open | closed |
| 4 | ring | open | closed |
| 5 | pinky | open | closed |

Verified: each digit actuated open↔closed independently on both hands (lateral
`thumb_aux` confirmed). In a full simultaneous fist the thumb/index stall at
~0.5–0.7 from finger collision (expected); each digit individually reaches 1.0.

### Touch-sensor mapping — `left_touch_force` / `right_touch_force`, `[5 ints, u16]`

Five fingertip normal-force channels (Modbus reg 4200). **Palms have no touch
sensor.** Idle ≈ 0, firm press saturates ≈ 2500.

| idx | fingertip | idle | firm press | measured peak (L · R) |
|---|---|---|---|---|
| 0 | thumb | ~0 | ~2500 | 1705 · 2500 |
| 1 | index | ~0 | ~2500 | 2500 · 2500 |
| 2 | middle | ~0 | ~2500 | 2500 · 2500 |
| 3 | ring | ~0 | ~2500 | 2500 · 2387 |
| 4 | pinky | ~0 | ~2500 | 2500 · 2500 |

10/10 fingertips registered clean single press events.

### Proximity-sensor mapping — `left_proximity` / `right_proximity`, `[5 ints, u16]`

Per-finger self-proximity, decoded from `touch_raw[16 + 2·i]` (only the
**second** register of each pair carries the signal). Idle ≈ 0, rising toward
the u16 ceiling (~65535) as an object nears the fingertip.

| idx | fingertip | register | idle | near (measured, L · R) |
|---|---|---|---|---|
| 0 | thumb | `touch_raw[16]` | 0 | 63,111 · 62,406 |
| 1 | index | `touch_raw[18]` | 0 | 57,335 · 63,808 |
| 2 | middle | `touch_raw[20]` | 0 | 61,643 · 64,967 |
| 3 | ring | `touch_raw[22]` | 0 | 59,517 · 65,440 |
| 4 | pinky | `touch_raw[24]` | 0 | 64,959 · 65,218 |

10/10 fingertips responded with a clean baseline-0 → near-saturation deflection.
