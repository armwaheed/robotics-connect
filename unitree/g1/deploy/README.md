# Unitree G1 — Policy deploy binding (`RobotIO`)

The G1 binding for the robot-agnostic on-hardware deploy ladder in
[`lib/policy_deploy.py`](../../../lib/policy_deploy.py). It implements the small `RobotIO`
interface over the G1's DDS, so the shared **de-risk ladder** drives the real robot.

> ⚠️ This commands real motors. Read [**SAFETY.md**](../../../SAFETY.md) first. The ladder wraps
> every motion stage in [`SafeStop`](../../../lib/safe_stop.py); **never `kill -9`** the process.

| Capability | G1 mechanism |
|---|---|
| state | `rt/lowstate` (HG `LowState_`: joint q/dq + IMU quat/gyro) + `rt/odommodestate` (base vel) |
| partial / fall-safe | `rt/arm_sdk` overlay with the weight blend (`motor_cmd[29].q`); legs on vendor balance |
| whole-body | all joints via `rt/lowcmd` after `MotionSwitcher.ReleaseMode` (sim-matched gains) |
| damp | **mode-aware**: `arm_sdk` weight→0 while overlaying, `rt/lowcmd` kp=0 once we hold the legs |
| abort | the handheld controller via [`G1Remote`](../controller/README.md) (any button latches) |
| vendor release | `MotionSwitcher` with a retry + **verify guard** — refuses to take over unless release confirmed |

## Use

```python
import sys; sys.path.insert(0, "lib")
from policy_deploy import DeployContract, PolicyDeploy
from g1_robot_io import G1RobotIO

contract = DeployContract.load("deploy_contract.json")     # dumped from the training env
io = G1RobotIO(iface="eth0", names=contract.action_joint_names)
io.connect()
dep = PolicyDeploy(contract, "policy.pt", io)              # torch loads the exported actor

dep.run_offline(target, steps=3)                          # rung 0: no motion (verify obs+mapping)
dep.run_partial(target, subset=ARM_NAMES, seconds=5)      # rung 1: arms only, fall-safe
dep.run_whole(target, seconds=3)                          # rung 2: GANTRY ONLY
```

Read-only connection check (no motion): `python g1_robot_io.py --iface eth0`.

## Joint indexing

The EDU motor table is indexed like the 29-DOF G1 — legs 0–11, `waist_yaw` 12, L-arm 15–19,
R-arm 22–26; the 6 absent EDU joints (13,14,20,21,27,28) are present-but-zero. The contract's
joint **names** map through `SDK_INDEX`, so the IsaacLab interleaved action order resolves to the
right motors. See the [deploy-policy skill](../../../skills/deploy-policy/SKILL.md).

## Status

Structure is import-validated and the ladder logic is unit-tested off-robot (via
`MockRobotIO` in `lib/test_policy_deploy.py`). The **DDS I/O paths need an on-hardware check** —
the mechanics are lifted from the eye-verified bed-reach run (`armwaheed/robots#3`
`g1_bedreach_deploy.py`), but this refactor itself has not yet been re-run on the robot.
