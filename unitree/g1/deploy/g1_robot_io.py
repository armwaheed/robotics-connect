"""Unitree G1 binding for the robot-agnostic policy-deploy ladder (``lib/policy_deploy.py``).

Implements :class:`RobotIO` over the G1's DDS interface so the shared de-risk ladder
(``run_offline`` / ``run_partial`` / ``run_whole``) drives the real robot:

  * **state** — ``rt/lowstate`` (HG ``LowState_``: joint q/dq + IMU quat/gyro) and
    ``rt/odommodestate`` (``SportModeState_``: base linear velocity);
  * **partial (fall-safe)** — joint targets via ``rt/arm_sdk`` with the weight overlay
    (``motor_cmd[29].q``), legs staying on the vendor balance controller;
  * **whole-body** — all joints via ``rt/lowcmd`` after ``MotionSwitcher.ReleaseMode``;
  * **damp** — mode-aware: ``rt/arm_sdk`` weight→0 while overlaying, ``rt/lowcmd`` kp=0
    once we've taken over the legs (so a damp never releases the legs into free-fall);
  * **abort** — the handheld controller via ``G1Remote`` (any button latches).

SAFETY: this commands real motors. Read ``../../../SAFETY.md``; the ladder wraps every motion
stage in ``SafeStop``, and you must NEVER ``kill -9`` the process (a hard kill latches the last
high-gain command → runaway). The robot must be SUPPORTED (gantry) for ``run_whole``.

Joint indexing: the G1 EDU motor table is indexed like the 29-DOF G1 (legs 0–11, waist_yaw 12,
L-arm 15–19, R-arm 22–26); the 6 absent EDU joints (13,14,20,21,27,28) are present-but-zero and
never in the 23-joint action set. The deploy contract's joint NAMES map through ``SDK_INDEX``.
"""
from __future__ import annotations

import importlib.util
import math
import sys
import time
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[3]
_LIB = _ROOT / "lib"


def _load(name: str, path: Path):
    """Load a module by file path under a unique name (no sys.path pollution — avoids the
    ``locomotion`` clash the repo guards against)."""
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, str(path))
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


# Pre-load safe_stop as ``safe_stop`` so policy_deploy's fallback ``from safe_stop import`` resolves.
_load("safe_stop", _LIB / "safe_stop.py")
_pd = _load("robotics_connect_policy_deploy", _LIB / "policy_deploy.py")
RobotIO = _pd.RobotIO
RobotState = _pd.RobotState
_g1remote = _load("g1_remote_deploy", _ROOT / "unitree/g1/controller/g1_remote.py")

# Unitree SDK motor index per joint name (standard G1 layout; verified live on the EDU).
SDK_INDEX = {
    "left_hip_pitch_joint": 0, "left_hip_roll_joint": 1, "left_hip_yaw_joint": 2,
    "left_knee_joint": 3, "left_ankle_pitch_joint": 4, "left_ankle_roll_joint": 5,
    "right_hip_pitch_joint": 6, "right_hip_roll_joint": 7, "right_hip_yaw_joint": 8,
    "right_knee_joint": 9, "right_ankle_pitch_joint": 10, "right_ankle_roll_joint": 11,
    "waist_yaw_joint": 12, "waist_roll_joint": 13, "waist_pitch_joint": 14,
    "left_shoulder_pitch_joint": 15, "left_shoulder_roll_joint": 16, "left_shoulder_yaw_joint": 17,
    "left_elbow_joint": 18, "left_wrist_roll_joint": 19, "left_wrist_pitch_joint": 20,
    "left_wrist_yaw_joint": 21,
    "right_shoulder_pitch_joint": 22, "right_shoulder_roll_joint": 23, "right_shoulder_yaw_joint": 24,
    "right_elbow_joint": 25, "right_wrist_roll_joint": 26, "right_wrist_pitch_joint": 27,
    "right_wrist_yaw_joint": 28,
}
ARM_SDK_WEIGHT_IDX = 29  # motor_cmd[29].q is the arm_sdk blend weight
MODE_PR = 0
DAMP_KD = 3.0


def _set_motor(mc, q, kp, kd, mode=None):
    """Fill one motor command (position target). ``mode=1`` enables the joint for rt/lowcmd;
    the rt/arm_sdk overlay does not set ``mode``."""
    if mode is not None:
        mc.mode = mode
    mc.q = float(q)
    mc.dq = 0.0
    mc.kp = float(kp)
    mc.kd = float(kd)
    mc.tau = 0.0


class G1RobotIO(RobotIO):
    """Concrete :class:`RobotIO` for the Unitree G1 over DDS."""

    def __init__(self, iface: str = "eth0", *, names=None) -> None:
        self._iface = iface
        # joints this binding reports/controls (defaults to all named G1 joints)
        self._names = list(names) if names else list(SDK_INDEX.keys())
        self._sdk = {n: SDK_INDEX[n] for n in self._names}
        self._ls = None          # latest rt/lowstate
        self._sms = None         # latest rt/odommodestate
        self._mode_machine = 0
        self._lowcmd_pub = None
        self._armsdk_pub = None
        self._low_cmd = None
        self._crc = None
        self._remote = None
        self._msc = None
        self._control = None     # "lowcmd" | "arm_sdk" | None — which authority we hold
        self._orig_mode = None

    # ── lifecycle ────────────────────────────────────────────────────────────
    def connect(self) -> None:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
        from unitree_sdk2py.utils.crc import CRC
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_

        ChannelFactoryInitialize(0, self._iface)
        ChannelSubscriber("rt/lowstate", LowState_).Init(self._on_ls, 10)
        ChannelSubscriber("rt/odommodestate", SportModeState_).Init(self._on_sms, 10)
        t = time.time()
        while (self._ls is None or self._sms is None) and time.time() - t < 4.0:
            time.sleep(0.02)
        if self._ls is None:
            raise RuntimeError("no rt/lowstate — robot on? iface correct?")
        if self._sms is None:
            raise RuntimeError("no rt/odommodestate — robot on?")
        self._mode_machine = int(self._ls.mode_machine)
        self._low_cmd = unitree_hg_msg_dds__LowCmd_()
        self._crc = CRC()
        self._remote = _g1remote.G1Remote(iface=self._iface, init_dds=False)
        self._remote.connect()

    def _on_ls(self, m):
        self._ls = m

    def _on_sms(self, m):
        self._sms = m

    def _lowcmd(self):
        if self._lowcmd_pub is None:
            from unitree_sdk2py.core.channel import ChannelPublisher
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
            self._lowcmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
            self._lowcmd_pub.Init()
        return self._lowcmd_pub

    def _armsdk(self):
        if self._armsdk_pub is None:
            from unitree_sdk2py.core.channel import ChannelPublisher
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
            self._armsdk_pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
            self._armsdk_pub.Init()
        return self._armsdk_pub

    # ── RobotIO: state ───────────────────────────────────────────────────────
    def read_state(self) -> RobotState:
        ms = self._ls.motor_state
        q = {n: float(ms[self._sdk[n]].q) for n in self._names}
        dq = {n: float(ms[self._sdk[n]].dq) for n in self._names}
        im = self._ls.imu_state
        quat = tuple(float(x) for x in im.quaternion)      # wxyz
        gyro = tuple(float(x) for x in im.gyroscope)
        base_lin = tuple(float(x) for x in self._sms.velocity)
        return RobotState(q=q, dq=dq, quat_wxyz=quat, gyro=gyro, base_lin_vel=base_lin)

    # ── RobotIO: publish ─────────────────────────────────────────────────────
    def publish_targets(self, targets: dict, gains: dict, weight=None) -> None:
        if weight is None:
            self._control = "lowcmd"
            self._publish_lowcmd(targets, gains)
        else:
            self._control = "arm_sdk"
            self._publish_armsdk(targets, gains, weight)

    def _publish_lowcmd(self, targets, gains):
        cmd = self._low_cmd
        cmd.mode_pr = MODE_PR
        cmd.mode_machine = self._mode_machine
        for n, q in targets.items():
            kp, kd = gains.get(n, (0.0, 0.0))
            _set_motor(cmd.motor_cmd[self._sdk[n]], q, kp, kd, mode=1)
        cmd.crc = self._crc.Crc(cmd)
        self._lowcmd().Write(cmd)

    def _publish_armsdk(self, targets, gains, weight):
        cmd = self._low_cmd
        cmd.motor_cmd[ARM_SDK_WEIGHT_IDX].q = float(weight)
        for n, q in targets.items():
            kp, kd = gains.get(n, (40.0, 1.5))
            _set_motor(cmd.motor_cmd[self._sdk[n]], q, kp, kd)
        cmd.crc = self._crc.Crc(cmd)
        self._armsdk().Write(cmd)

    # ── RobotIO: damp (mode-aware) ───────────────────────────────────────────
    def damp_once(self) -> None:
        if self._control == "arm_sdk":
            # release the overlay: weight 0 hands the arms back to the vendor balance controller.
            cmd = self._low_cmd
            cmd.motor_cmd[ARM_SDK_WEIGHT_IDX].q = 0.0
            cmd.crc = self._crc.Crc(cmd)
            self._armsdk().Write(cmd)
        else:
            # we hold the legs (or released the vendor) → low-level compliant damp (kp=0).
            state = self.read_state()
            cmd = self._low_cmd
            cmd.mode_pr = MODE_PR
            cmd.mode_machine = self._mode_machine
            for n in self._names:
                _set_motor(cmd.motor_cmd[self._sdk[n]], state.q[n], 0.0, DAMP_KD, mode=1)
            cmd.crc = self._crc.Crc(cmd)
            self._lowcmd().Write(cmd)

    # ── RobotIO: abort ───────────────────────────────────────────────────────
    def abort_tripped(self) -> bool:
        return self._remote.aborted()

    def arm_abort(self, timeout_s: float = 8.0) -> bool:
        return self._remote.wait_until_armed(timeout_s)

    # ── RobotIO: vendor controller (whole-body) ──────────────────────────────
    def release_vendor(self) -> bool:
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
        if self._msc is None:
            self._msc = MotionSwitcherClient()
            self._msc.SetTimeout(5.0)
            self._msc.Init()

        def check():
            for _ in range(12):
                code, data = self._msc.CheckMode()
                if code == 0:
                    return data or {}
                time.sleep(0.3)
            return None

        data = check()
        if data is None:
            return False  # couldn't read the mode at all → don't take over
        self._orig_mode = data.get("name")
        name = self._orig_mode
        tries = 0
        while name and tries < 12:
            self._msc.ReleaseMode()
            time.sleep(0.5)
            data = check()
            name = (data or {}).get("name")
            tries += 1
        if name:
            return False  # could not confirm release → refuse to take over
        self._control = "lowcmd"  # after release, rt/lowcmd is the authority (damp goes low-level)
        return True

    def reengage_vendor(self) -> None:
        if self._msc is not None and self._orig_mode:
            self._msc.SelectMode(self._orig_mode)

    def shutdown(self) -> None:
        try:
            if self._remote is not None:
                self._remote.shutdown()
        except Exception:
            pass


def _read_only_pose(iface: str = "eth0") -> None:
    """CLI: stream IMU tilt + base velocity read-only (no motion) as a connection sanity check."""
    io = G1RobotIO(iface=iface)
    io.connect()
    print("[G1RobotIO] connected — streaming read-only state for 4 s (no motion)")
    end = time.time() + 4.0
    while time.time() < end:
        s = io.read_state()
        rpy = [round(math.degrees(x), 1) for x in _quat_to_rpy(s.quat_wxyz)]
        print(f"  rpy(deg)={rpy}  base_lin_vel={[round(v, 3) for v in s.base_lin_vel]}")
        time.sleep(0.5)
    io.shutdown()


def _quat_to_rpy(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="G1RobotIO read-only connection check (no motion).")
    ap.add_argument("--iface", default="eth0")
    _read_only_pose(ap.parse_args().iface)
