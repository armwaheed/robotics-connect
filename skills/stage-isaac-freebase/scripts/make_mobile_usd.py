"""Author a MOBILE-base (free-base) override USD for a humanoid whose stock USD bakes a world pin.

**The open problem.** Many humanoid USDs ship fixed-base + gravity-off for tabletop manipulation, and
bake the ``ArticulationRootAPI`` onto a ``root_joint`` ``PhysicsFixedJoint`` that pins the pelvis to
the world. Setting ``fix_root_link=False`` only *disables* that joint, after which Isaac can no longer
resolve the articulation root (it still looks for it on the joint) → ``Failed to create articulation``.
For the Unitree G1 with 5-finger Inspire hands this is NVIDIA forum thread **370590** (unanswered).

**The reusable fix.** Write a tiny LOCAL override layer that references the stock USD and:
  * deactivates the baked ``root_joint`` (removes the world pin), and
  * applies the ``ArticulationRootAPI`` (+ ``PhysxArticulationAPI``) to the pelvis/root LINK,
so the manager-based spawn path builds a true floating articulation. The output references the stock
USD (no inlined meshes → a ~1 KB file) and needs the same asset server the stock config already uses.

This is robot-agnostic: it detects where the articulation root currently lives and which baked fixed
joint pins it, so it fixes the Unitree G1 (``root_joint`` → ``pelvis``) or any other humanoid USD with
the same affliction (``--root-link`` names that robot's base link).

Run once (from the IsaacLab dir; LD_PRELOAD is mandatory on aarch64):
  cd ~/workspaces/git/IsaacLab && export LD_PRELOAD=$LD_PRELOAD:/lib/aarch64-linux-gnu/libgomp.so.1 \
    && ./isaaclab.sh -p make_mobile_usd.py \
        --src "${ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/G1/g1_29dof_inspire_hand.usd" \
        --out ./assets/g1_inspire_mobile.usd --root-link pelvis
"""

from __future__ import annotations

import argparse

parser = argparse.ArgumentParser(description="Author a free-base override USD for a humanoid.")
parser.add_argument("--src", required=True, help="Stock USD (the one that bakes a world pin).")
parser.add_argument("--out", required=True, help="Output override USD path.")
parser.add_argument("--robot-prim", default="/Robot", help="Default prim path of the robot in the USD.")
parser.add_argument("--root-link", default="pelvis",
                    help="Base link the articulation root should move onto (e.g. 'pelvis', 'base_link').")
parser.add_argument("--root-joint", default=None,
                    help="Name of the baked world-pin fixed joint to deactivate. Auto-detected if omitted.")
args = parser.parse_args()

from isaaclab.app import AppLauncher  # noqa: E402

app_launcher = AppLauncher({"headless": True})
simulation_app = app_launcher.app

import os  # noqa: E402

from pxr import PhysxSchema, Usd, UsdPhysics  # noqa: E402


def prims_with_articulation_api(stage: Usd.Stage) -> list[str]:
    return [p.GetPath().pathString for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]


def baked_world_pin(stage: Usd.Stage, robot_prim: str) -> str | None:
    """Find a baked world-pin: a PhysicsFixedJoint under the robot whose body0 is unset/world (so it
    pins the robot to the world). Returns its prim path, or None."""
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.FixedJoint):
            continue
        if robot_prim not in prim.GetPath().pathString:
            continue
        joint = UsdPhysics.FixedJoint(prim)
        body0 = joint.GetBody0Rel().GetTargets()
        # A world pin has no body0 (or body0 == world) — one side is ground.
        if not body0:
            return prim.GetPath().pathString
    return None


def main() -> int:
    src, out = args.src, os.path.abspath(args.out)
    robot_prim, root_link = args.robot_prim, args.root_link

    # 1) Inspect the stock asset.
    src_stage = Usd.Stage.Open(src)
    default_prim = src_stage.GetDefaultPrim()
    print(f"[usd] source default prim: {default_prim.GetPath()}")
    print(f"[usd] source prims carrying ArticulationRootAPI: {prims_with_articulation_api(src_stage)}")
    root_joint = args.root_joint or "root_joint"
    detected = baked_world_pin(src_stage, robot_prim)
    if detected:
        print(f"[usd] detected baked world-pin fixed joint: {detected}")
        if args.root_joint is None:
            root_joint = os.path.basename(detected)
    else:
        print(f"[usd] WARNING: no baked world-pin auto-detected; using --root-joint '{root_joint}'.")

    root_joint_path = f"{robot_prim}/{root_joint}"
    root_link_path = f"{robot_prim}/{root_link}"

    # 2) Author the local override layer.
    if os.path.exists(out):
        os.remove(out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    stage = Usd.Stage.CreateNew(out)
    robot = stage.DefinePrim(robot_prim, "Xform")
    robot.GetReferences().AddReference(src)
    stage.SetDefaultPrim(robot)

    stage.OverridePrim(root_joint_path).SetActive(False)          # remove the world pin

    pelvis = stage.OverridePrim(root_link_path)                   # articulation root → base link
    UsdPhysics.ArticulationRootAPI.Apply(pelvis)
    PhysxSchema.PhysxArticulationAPI.Apply(pelvis)

    stage.GetRootLayer().Save()

    # 3) Verify the composed result.
    check = Usd.Stage.Open(out)
    rj = check.GetPrimAtPath(root_joint_path)
    rj_active = rj.IsActive() if rj and rj.IsValid() else False
    link_is_root = check.GetPrimAtPath(root_link_path).HasAPI(UsdPhysics.ArticulationRootAPI)
    print(f"[usd] wrote {out}")
    print(f"[usd] composed {root_joint} active = {rj_active} (want False)")
    print(f"[usd] composed {root_link} has ArticulationRootAPI = {link_is_root} (want True)")
    print(f"[usd] composed prims carrying ArticulationRootAPI: {prims_with_articulation_api(check)}")
    ok = (not rj_active) and link_is_root
    print(f"[usd] OVERALL: {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


rc = main()
simulation_app.close()
import sys  # noqa: E402

sys.exit(rc)
