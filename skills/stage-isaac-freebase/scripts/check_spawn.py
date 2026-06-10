"""Spawn-validation probe for a free-base humanoid (run BEFORE building the RL env).

Confirms, for the robot described by a descriptor, that the mobile-base USD + deploy gains actually
yield a stable free-base articulation — the cheap check that catches a botched free-base fix or
collapse-inducing gains before you spend a training run.

Checks (printed PASS/FAIL):
  1. ``is_fixed_base`` is False                  -> the baked world pin was removed (free base)
  2. every action joint + the EE/foot/pelvis bodies resolve by name
  3. with gravity ON and zero action, the robot STANDS for 3 s (pelvis z stays in band) -> the deploy
     gains hold it up, the spawn height is right

Run (from the IsaacLab dir; LD_PRELOAD mandatory on aarch64):
  cd ~/workspaces/git/IsaacLab && export LD_PRELOAD=$LD_PRELOAD:/lib/aarch64-linux-gnu/libgomp.so.1 \
    && ./isaaclab.sh -p check_spawn.py \
        --descriptor ../../discover-robot/descriptors/unitree_g1_edu.json \
        --mobile-usd ./assets/g1_inspire_mobile.usd

Lifted + generalized from armwaheed/robots#2 ``rl/check_spawn.py``.
"""

from __future__ import annotations

import argparse

parser = argparse.ArgumentParser(description="Validate a free-base humanoid spawn.")
parser.add_argument("--descriptor", required=True, help="Path to the robot descriptor JSON.")
parser.add_argument("--mobile-usd", default=None, help="Free-base USD from make_mobile_usd.py.")
parser.add_argument("--pelvis", default="pelvis", help="Pelvis/root body name.")
parser.add_argument("--feet", default=".*_ankle_roll_link", help="Foot body name regex.")
args = parser.parse_args()

from isaaclab.app import AppLauncher  # noqa: E402

app_launcher = AppLauncher({"headless": True})
simulation_app = app_launcher.app

import os  # noqa: E402
import sys  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import robot_cfg as rc  # noqa: E402


def main() -> int:
    descriptor = rc.load_descriptor(args.descriptor)
    act_joints = rc.action_joints(descriptor)
    ee = descriptor["effectors"].get("ee_links", {})

    sim = SimulationContext(sim_utils.SimulationCfg(dt=0.005, device="cuda:0"))
    sim.set_camera_view([2.5, 2.5, 2.0], [0.0, 0.0, 0.8])
    sim_utils.GroundPlaneCfg().func("/World/ground", sim_utils.GroundPlaneCfg())
    sim_utils.DomeLightCfg(intensity=1500.0).func("/World/Light", sim_utils.DomeLightCfg(intensity=1500.0))

    robot = Articulation(rc.make_g1_cfg(descriptor, prim_path="/World/Robot", mobile_usd=args.mobile_usd))
    sim.reset()
    ok = True

    # 1) Free base?
    free = not bool(robot.is_fixed_base)
    print(f"[check] is_fixed_base={robot.is_fixed_base}  ->  free base: {'PASS' if free else 'FAIL'}")
    ok = ok and free

    # 2) Action joints + key bodies resolve?
    joint_ids, _ = robot.find_joints(act_joints, preserve_order=False)
    n = len(joint_ids)
    print(f"[check] resolved {n}/{len(act_joints)} action joints  ->  {'PASS' if n == len(act_joints) else 'FAIL'}")
    ok = ok and (n == len(act_joints))
    for label, expr in [("right EE", ee.get("right")), ("left EE", ee.get("left")),
                        ("pelvis", args.pelvis), ("feet", args.feet)]:
        if not expr:
            continue
        ids, names = robot.find_bodies(expr, preserve_order=False)
        print(f"[check]   body '{expr}' ({label}) -> {names}")
        ok = ok and (len(ids) >= 1)
    print(f"[check] total joints={robot.num_joints}, total bodies={robot.num_bodies}, locked={rc.locked_joints(descriptor)}")

    # 3) Stands under gravity with zero action (holds default targets) for 3 s.
    default_targets = robot.data.default_joint_pos.clone()
    z0 = robot.data.root_pos_w[0, 2].item()
    zmin = zmax = z0
    for _ in range(600):  # 3 s at dt=0.005
        robot.set_joint_position_target(default_targets)
        robot.write_data_to_sim()
        sim.step()
        robot.update(0.005)
        z = robot.data.root_pos_w[0, 2].item()
        zmin, zmax = min(zmin, z), max(zmax, z)
    zf = robot.data.root_pos_w[0, 2].item()
    stood = (0.55 < zf < 0.95) and (zmin > 0.40)
    print(f"[check] pelvis z: spawn={z0:.3f} final={zf:.3f} min={zmin:.3f} max={zmax:.3f}  "
          f"->  stands: {'PASS' if stood else 'FAIL'}")
    ok = ok and stood

    print(f"[check] OVERALL: {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


rc_code = main()
simulation_app.close()
sys.exit(rc_code)
