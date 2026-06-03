#!/usr/bin/env python3
"""
Bridge: Brainco Revo2 hands <-> TCP socket for hand control.

Pure pyserial implementation — no ROS2, no stark_node, no Rust SDK.

V2 HARDWARE IS Revo2Touch (hardware_type=4)
-------------------------------------------
`modbus_get_device_info` reports hardware_type=4 (STARK_HARDWARE_TYPE_REVO2_TOUCH)
and firmware 1.0.14.U. The installed libbc_stark_sdk.so (0.4.3) hard-codes the
touch APIs as "deprecated for current firmware" and returns empty — but the
newer 0.8.1 SDK accepts the device and successfully reads register 4200 for
per-finger normal/tangential force and proximity.

We bypass the SDK entirely and talk to the same Modbus registers via pyserial,
matching what 0.8.1's `modbus_get_touch_status` does under the hood:

  * Enable : WRITE holding registers @ 4000, data = [1, 1, 1, 1, 1]  (one per finger)
  * Calib. : WRITE holding registers @ 4010, data = [1, 1, 1, 1, 1]
  * Status : READ  input    registers @ 4200, count = 30

Register 4200 layout (u16 × 30, verified on-robot 2026-04-14):

  regs[ 0..14] — 3 values per finger, interleaved f0..f4:
      reg[3*i + 0] = normal_force           (uint16, 0 ≈ idle, ≳500 = firm press)
      reg[3*i + 1] = tangential_force       (uint16)
      reg[3*i + 2] = tangential_direction   (uint16)

  regs[15..24] — 2 values per finger, interleaved f0..f4:
      reg[15 + 2*i + 0] = self_proximity_lo (uint16, low half of u32)
      reg[15 + 2*i + 1] = self_proximity_hi (uint16, high half of u32)

  regs[25..29] — per-finger frame counter / status, not useful, ignore.

Finger order (touch indices 0..4): thumb, index, middle, ring, pinky.

MOTOR STATUS (unchanged)
------------------------
  READ  input  registers @ 2000 (count 24)  — motor status
        [0:6]   = positions (uint16, range 0-1000)
        [6:12]  = speeds    (int16)
        [12:18] = currents  (int16, signed)
        [18:24] = states    (uint16)

  WRITE holding registers @ 1070 (count 3)  — finger positions
        Each register packs 2 finger positions as bytes.  The device's
        per-register bit layout is the REVERSE of what naive big-endian
        packing produces: the HIGH byte is the SECOND finger in each
        documented pair, the LOW byte is the FIRST.  Empirically verified
        on the robot on 2026-04-21:
        commanding pos[0]=1.0 (with the old naive packing) drove the
        thumb to claw-lateral + extended — i.e. the HIGH byte of reg[0]
        is thumb_aux, NOT thumb_curl.  The swap below restores symmetry
        with the READ side (register 2000, one-per-motor, natural order).

          reg[0] = (pos[1] << 8) | pos[0]   # high=thumb_aux, low=thumb_curl
          reg[1] = (pos[3] << 8) | pos[2]   # high=middle,    low=index
          reg[2] = (pos[5] << 8) | pos[4]   # high=pinky,     low=ring

        Each finger position is 0-100 (uint8 percent closed).

Motor finger order for pos[0..5] (write AND read, after the fix above):
    thumb_curl, thumb_aux, index, middle, ring, pinky.
Position range: 0.0 = open, 1.0 = closed (commanded), reported scale 0-1000.
For thumb_aux specifically: 0.0 = slap (thumb abducted away from palm),
1.0 = claw (thumb opposed across palm toward the fingertips).

Run:

    sudo chmod 666 /dev/ttyUSB1 /dev/ttyUSB2
    conda activate g1brainco          # or any env with pyserial
    python ~/robotics-connect/brainco_touch/brainco_bridge.py

Protocol: newline-delimited JSON over TCP on 127.0.0.1:9877
  Read state:   {"cmd": "get"}
    -> {
         "left":           [6 floats, 0-1],   # actual motor positions (normalised)
         "right":          [6 floats, 0-1],
         "left_currents":  [6 floats, a.u.],  # signed motor currents / 100
         "right_currents": [6 floats, a.u.],
         "left_touch":     [5 floats, 0-1],   # normal_force per finger / LOAD_NORM
         "right_touch":    [5 floats, 0-1],   # finger order: thumb,index,middle,ring,pinky
         "left_touch_force":  [5 ints, u16],  # raw normal_force (0..65535)
         "right_touch_force": [5 ints, u16],
         "left_touch_raw":  [30 ints, u16],   # full register block at addr 4200
         "right_touch_raw": [30 ints, u16],
         "left_touch60_raw":  [60 ints, u16], # rich per-zone block at addr 4300
         "right_touch60_raw": [60 ints, u16], # used for first-contact servo
         "left_lag":       [6 floats, 0-1],   # commanded - actual position
         "right_lag":      [6 floats, 0-1],
         "left_touch_ok":  bool,              # True if touch enable succeeded
         "right_touch_ok": bool,
       }

  Send command: {"cmd": "set", "left": [6 floats 0-1], "right": [6 floats 0-1]}
    -> {"ok": true}
"""

import argparse
import json
import logging
import socket
import struct
import threading
import time

import serial

log = logging.getLogger("brainco_bridge")

HOST = "127.0.0.1"
PORT = 9877

# Defaults for Brainco Revo2 V2
DEFAULT_PORT_L = "/dev/ttyUSB1"
DEFAULT_PORT_R = "/dev/ttyUSB2"
DEFAULT_BAUD = 460800
SLAVE_ID_L = 0x7E
SLAVE_ID_R = 0x7F

# Modbus register addresses
REG_MOTOR_STATUS_ADDR = 2000
REG_MOTOR_STATUS_COUNT = 24
REG_FINGER_POS_ADDR = 1070   # write 3 packed uint16 = 6 finger positions

# Touch sensor registers (verified on Revo2Touch, firmware 1.0.14.U)
REG_TOUCH_ENABLE_ADDR = 4000    # write 5 holding regs = one per finger
REG_TOUCH_CALIBRATE_ADDR = 4010 # write 5 holding regs = one per finger
REG_TOUCH_STATUS_ADDR = 4200    # read 30 input regs
REG_TOUCH_STATUS_COUNT = 30

# Rich per-zone touch block (visual-tactile first-contact bring-up).
# Empirically the firmware exposes 60 u16 of richer per-
# zone touch sensor data here.  Every finger press on either hand fires
# ~36 of these 60 regs above |Δ|>1024 from idle baseline — including
# thumb and pinky which the 30-reg block at 4200 misses.  Encoding: each
# u16's HIGH byte is the active value (0..255); LOW byte is 0 at idle.
# Per-finger mapping is NOT static (mechanical cross-talk through the
# rigid hand chassis), so consumers should use this as a single per-hand
# "any-finger-touched" signal computed as max(|reg - baseline|).
REG_TOUCH60_ADDR = 4300
REG_TOUCH60_COUNT = 60

# Position scale: raw register values from motor status are 0-1000
POSITION_RAW_MAX = 1000.0

# Touch detection tuning.
# LAG_NORM: position-lag fallback (used only if real touch sensors fail to enable)
LAG_NORM = 0.10  # 10% lag = touch=1.0

# LOAD_NORM: normalise per-finger normal force to 0..1 for the legacy touch field.
# On-robot observation shows idle≈0, firm fingertip press peaks ~1500-2500 raw
# units, so 500 gives a clean "contact detected" level with room for harder grips.
LOAD_NORM = 500.0

# Map motor-index [0..5] to touch-index [0..4].
# thumb=0, thumb_aux=1 -> thumb, index=2, middle=3, ring=4, pinky=5
_MOTOR_TO_TOUCH = {0: 0, 1: 0, 2: 1, 3: 2, 4: 3, 5: 4}

POLL_INTERVAL = 0.04  # 25 Hz


# ---------------------------------------------------------------------------
# Modbus RTU helpers
# ---------------------------------------------------------------------------

def _modbus_crc(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def _read_input_registers(ser, slave_id, addr, count):
    """Read input registers (function 0x04). Returns list of uint16 or None."""
    msg = struct.pack(">BBHH", slave_id, 0x04, addr, count)
    msg += struct.pack("<H", _modbus_crc(msg))
    ser.reset_input_buffer()
    ser.write(msg)
    expected = 3 + count * 2 + 2
    resp = ser.read(expected)
    if len(resp) < expected:
        return None
    if _modbus_crc(resp[:-2]) != struct.unpack("<H", resp[-2:])[0]:
        return None
    byte_count = resp[2]
    return list(struct.unpack(">" + "H" * count, resp[3:3 + byte_count]))


def _write_finger_positions(ser, slave_id, positions_0_100):
    """Write 6 finger positions (each 0-100) packed into 3 holding registers.

    See the module docstring's "WRITE holding registers @ 1070" section —
    the device's per-register layout has the HIGH byte as the SECOND
    documented finger in each pair and the LOW byte as the FIRST, so
    thumb_aux / middle / pinky go into the HIGH byte and thumb_curl /
    index / ring into the LOW byte of their respective register.
    """
    p = [max(0, min(100, int(v))) for v in positions_0_100]
    regs = [
        (p[1] << 8) | p[0],
        (p[3] << 8) | p[2],
        (p[5] << 8) | p[4],
    ]
    count = 3
    msg = struct.pack(">BBHHB", slave_id, 0x10, REG_FINGER_POS_ADDR, count, count * 2)
    for r in regs:
        msg += struct.pack(">H", r & 0xFFFF)
    msg += struct.pack("<H", _modbus_crc(msg))
    ser.reset_input_buffer()
    ser.write(msg)
    # Response: slave + func + addr(2) + count(2) + crc(2) = 8 bytes
    resp = ser.read(8)
    return len(resp) == 8


def _write_multiple_registers(ser, slave_id, addr, values):
    """Modbus function 0x10 — write multiple holding registers.  Returns True on success."""
    count = len(values)
    byte_count = count * 2
    msg = struct.pack(">BBHHB", slave_id, 0x10, addr, count, byte_count)
    for v in values:
        msg += struct.pack(">H", int(v) & 0xFFFF)
    msg += struct.pack("<H", _modbus_crc(msg))
    ser.reset_input_buffer()
    ser.write(msg)
    resp = ser.read(8)  # echo: slave + func + addr(2) + count(2) + crc(2)
    if len(resp) < 8:
        return False
    if _modbus_crc(resp[:-2]) != struct.unpack("<H", resp[-2:])[0]:
        return False
    return True


# ---------------------------------------------------------------------------
# Hand state reader
# ---------------------------------------------------------------------------

class HandReader:
    """Polls one hand over modbus and exposes the latest state."""

    def __init__(self, port, slave_id, baud, name):
        self.name = name
        self._slave_id = slave_id
        self._port_lock = threading.Lock()  # serialise port access
        self._state_lock = threading.Lock()
        self._ser = serial.Serial(port, baud, timeout=0.3)

        self._positions = [0.0] * 6
        self._currents = [0.0] * 6
        self._touch = [0.0] * 5            # 0..1 normalised per finger
        self._touch_force = [0] * 5         # raw normal_force per finger
        self._touch_raw = [0] * REG_TOUCH_STATUS_COUNT  # full 30-reg block at 4200
        self._touch60_raw = [0] * REG_TOUCH60_COUNT  # rich per-zone block at 4300
        self._lag = [0.0] * 6
        self._commanded = [0.0] * 6
        self._touch_ok = False
        self._running = True

        # Force hands fully open at startup
        with self._port_lock:
            _write_finger_positions(self._ser, self._slave_id, [0] * 6)

        # Enable touch sensors and calibrate the idle baseline in firmware.
        # Without enable, register 4200 returns [0]*25 + counters.
        self._enable_touch_sensors()

        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()
        log.info("[%s] started on %s (slave=0x%02x) touch_ok=%s",
                 name, port, slave_id, self._touch_ok)

    def _enable_touch_sensors(self):
        """Enable + calibrate all 5 touch sensor channels on this hand."""
        with self._port_lock:
            ok_en = _write_multiple_registers(
                self._ser, self._slave_id, REG_TOUCH_ENABLE_ADDR, [1] * 5
            )
            if not ok_en:
                log.warning("[%s] touch ENABLE failed — falling back to position-lag", self.name)
                return
            time.sleep(0.3)
            ok_cal = _write_multiple_registers(
                self._ser, self._slave_id, REG_TOUCH_CALIBRATE_ADDR, [1] * 5
            )
            if not ok_cal:
                log.warning("[%s] touch CALIBRATE failed (continuing anyway)", self.name)
            time.sleep(0.5)
        self._touch_ok = True

    @staticmethod
    def _parse_touch_regs(regs):
        """Unpack 30 u16 input registers from addr 4200 into per-finger fields.

        Layout verified on Revo2Touch firmware 1.0.14.U.  First confirmed
        on 2026-04-14 by RIGHT-hand single-finger press
        tests: pressing LEFT thumb lit reg[0]=2500, RIGHT thumb lit
        reg[0]=2500, RIGHT ring lit reg[9]=2500.  Expanded on 2026-04-21
        on the LEFT hand — all five fingers pressed one at a time cleanly lit
        touch indices 0..4 in documented order (thumb → index → middle
        → ring → pinky), confirming that the
        register-4200 layout does NOT inherit the pair-swap bug that
        afflicted WRITE register 1070 packing.

          regs[0..14]  = 3 per finger, interleaved (norm, tang, tang_dir)
          regs[15..24] = 2 per finger, interleaved (self_prox_lo, self_prox_hi)
          regs[25..29] = counter/status, not used

        The firmware writes 0xFFFF as a "no data" sentinel on the force
        fields when a finger is not pressed (mostly tang_dir).  We mask
        those to 0 so downstream callers don't see spurious ~max values.
        Proximity and counter registers can legitimately reach ~65535 so
        they're left alone.
        """
        def mask(v):
            return 0 if v == 0xFFFF else v
        normal = [mask(regs[3 * i + 0]) for i in range(5)]
        tangential = [mask(regs[3 * i + 1]) for i in range(5)]
        tang_dir = [mask(regs[3 * i + 2]) for i in range(5)]
        prox_lo = [regs[15 + 2 * i + 0] for i in range(5)]
        prox_hi = [regs[15 + 2 * i + 1] for i in range(5)]
        return normal, tangential, tang_dir, prox_lo, prox_hi

    def _poll_loop(self):
        consecutive_failures = 0
        while self._running:
            with self._port_lock:
                regs = _read_input_registers(
                    self._ser, self._slave_id,
                    REG_MOTOR_STATUS_ADDR, REG_MOTOR_STATUS_COUNT,
                )
                touch_regs = None
                touch60_regs = None
                if self._touch_ok and regs is not None:
                    touch_regs = _read_input_registers(
                        self._ser, self._slave_id,
                        REG_TOUCH_STATUS_ADDR, REG_TOUCH_STATUS_COUNT,
                    )
                    # Rich per-zone block — used for first-contact
                    # / any-finger-touched detection.
                    touch60_regs = _read_input_registers(
                        self._ser, self._slave_id,
                        REG_TOUCH60_ADDR, REG_TOUCH60_COUNT,
                    )
            if regs is None:
                consecutive_failures += 1
                if consecutive_failures % 20 == 0:
                    log.warning("[%s] %d consecutive read failures",
                                self.name, consecutive_failures)
                time.sleep(POLL_INTERVAL)
                continue
            consecutive_failures = 0

            positions_raw = regs[0:6]
            currents_raw = [
                v if v < 32768 else v - 65536 for v in regs[12:18]
            ]
            positions_01 = [min(1.0, p / POSITION_RAW_MAX) for p in positions_raw]

            # Parse touch registers if the real sensors are online.  Fall back to
            # position-lag on any failure so callers still get *some* touch.
            real_touch = None
            normal_force = [0] * 5
            if touch_regs is not None and len(touch_regs) >= REG_TOUCH_STATUS_COUNT:
                normal_force, _tang, _tdir, _plo, _phi = self._parse_touch_regs(touch_regs)
                real_touch = [min(1.0, nf / LOAD_NORM) for nf in normal_force]

            with self._state_lock:
                self._positions = positions_01
                self._currents = [c / 100.0 for c in currents_raw]
                self._lag = [
                    max(0.0, self._commanded[i] - positions_01[i])
                    for i in range(6)
                ]
                if real_touch is not None:
                    self._touch = real_touch
                    self._touch_force = list(normal_force)
                    self._touch_raw = list(touch_regs)
                if touch60_regs is not None and len(touch60_regs) >= REG_TOUCH60_COUNT:
                    self._touch60_raw = list(touch60_regs)
                if real_touch is None:
                    lag_touch = [0.0] * 5
                    for motor_i, touch_i in _MOTOR_TO_TOUCH.items():
                        s = min(1.0, self._lag[motor_i] / LAG_NORM)
                        if s > lag_touch[touch_i]:
                            lag_touch[touch_i] = s
                    self._touch = lag_touch

            time.sleep(POLL_INTERVAL)

    def get_state(self):
        with self._state_lock:
            return (
                list(self._positions),
                list(self._currents),
                list(self._touch),
                list(self._lag),
                list(self._touch_force),
                list(self._touch_raw),
                list(self._touch60_raw),
                bool(self._touch_ok),
            )

    def set_positions(self, positions_01):
        """Send finger positions (0-1 scale) to the hand."""
        clamped = [max(0.0, min(1.0, float(p))) for p in positions_01]
        positions_0_100 = [int(p * 100) for p in clamped]
        with self._state_lock:
            self._commanded = clamped
        with self._port_lock:
            _write_finger_positions(self._ser, self._slave_id, positions_0_100)

    def stop(self):
        self._running = False
        self._thread.join(timeout=2)
        try:
            self._ser.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# TCP bridge server
# ---------------------------------------------------------------------------

def _handle_client(conn, left, right):
    buf = ""
    try:
        while True:
            data = conn.recv(1024)
            if not data:
                break
            buf += data.decode("utf-8")
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if not line:
                    continue
                req = json.loads(line)
                if req["cmd"] == "get":
                    lp, lc, lt, ll, lf, lraw, lraw60, lok = left.get_state()
                    rp, rc, rt, rl, rf, rraw, rraw60, rok = right.get_state()
                    resp = json.dumps({
                        "left": lp,
                        "right": rp,
                        "left_currents": lc,
                        "right_currents": rc,
                        "left_touch": lt,
                        "right_touch": rt,
                        "left_touch_force": lf,
                        "right_touch_force": rf,
                        "left_touch_raw": lraw,
                        "right_touch_raw": rraw,
                        "left_touch60_raw": lraw60,
                        "right_touch60_raw": rraw60,
                        "left_touch_ok": lok,
                        "right_touch_ok": rok,
                        "left_lag": ll,
                        "right_lag": rl,
                    }) + "\n"
                    conn.sendall(resp.encode("utf-8"))
                elif req["cmd"] == "set":
                    left.set_positions(req["left"])
                    right.set_positions(req["right"])
                    conn.sendall(json.dumps({"ok": True}).encode("utf-8") + b"\n")
    except Exception as e:
        log.warning("[bridge] client error: %s", e)
    finally:
        conn.close()


def _serve(left, right, port=PORT):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, port))
    server.listen(5)
    log.info("[bridge] TCP server listening on %s:%d", HOST, port)
    while True:
        conn, addr = server.accept()
        log.info("[bridge] connection from %s", addr)
        t = threading.Thread(
            target=_handle_client, args=(conn, left, right), daemon=True,
        )
        t.start()


def main():
    parser = argparse.ArgumentParser(
        description="Brainco Revo2 direct-modbus bridge (no ROS2 / no stark_node / no SDK)",
    )
    parser.add_argument("--port-l", default=DEFAULT_PORT_L)
    parser.add_argument("--port-r", default=DEFAULT_PORT_R)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--tcp-port", type=int, default=PORT)
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    log.info("Brainco bridge — pure pyserial modbus")
    log.info("Left  hand: %s slave=0x%02x", args.port_l, SLAVE_ID_L)
    log.info("Right hand: %s slave=0x%02x", args.port_r, SLAVE_ID_R)

    left = HandReader(args.port_l, SLAVE_ID_L, args.baud, "left")
    right = HandReader(args.port_r, SLAVE_ID_R, args.baud, "right")

    try:
        _serve(left, right, args.tcp_port)
    except KeyboardInterrupt:
        log.info("Shutting down...")
    finally:
        left.stop()
        right.stop()


if __name__ == "__main__":
    main()
