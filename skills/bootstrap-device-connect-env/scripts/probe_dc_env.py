#!/usr/bin/env python3
"""Probe the Python environments on a robot/compute host and recommend how to run Device Connect.

THE RECURRING PROBLEM this resolves (see the skill's SKILL.md): ``device-connect-edge`` requires
**Python >= 3.11**, but most humanoid vendor SDKs pin an OLDER Python (Unitree G1 = 3.10, many
Jetson / ROS 2 Humble stacks = 3.8/3.10) and carry native deps that are painful to rebuild
(CycloneDDS, vendor wheels). So ``pip install device-connect-edge`` into the SDK env fails with
"No matching distribution", and forcing the SDK onto 3.11 risks bricking a working install.

This script gathers the facts an agent needs to DECIDE (it does not change anything):
  * enumerate every Python interpreter it can find (conda envs, venvs, system),
  * for each: Python version, whether the robot's SDK imports there, whether device-connect-edge
    imports there, and whether it is >= the device-connect-edge floor,
  * then print a RECOMMENDATION:
      - UNIFIED  : one env already has BOTH the SDK and Python >= floor -> install DC there.
      - BRIDGED  : the SDK env is too old -> keep it, run DC in a separate >= floor env, and have
                   the DC sidecar delegate hardware calls (speak/move/read) to the SDK env via a
                   subprocess/IPC bridge (the correct architecture for a vendor-pinned SDK).

Pure stdlib; run it with any python3:
    python3 probe_dc_env.py --sdk-module unitree_sdk2py
    python3 probe_dc_env.py --sdk-module unitree_sdk2py --json
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import shutil
import subprocess
import sys
from typing import Dict, List, Optional

# device-connect-edge's Python floor. Kept as data so the agent can bump it when the SDK moves.
DC_PYTHON_FLOOR = (3, 11)
DC_PACKAGE = "device-connect-edge"

# Common humanoid/robot SDK import names — pass --sdk-module to be explicit.
KNOWN_SDK_MODULES = ["unitree_sdk2py", "unitree_sdk2_python", "rclpy", "mujoco", "isaacsim"]


def _find_interpreters() -> List[str]:
    """Every python interpreter we can find: conda envs, venvs, the current one, system."""
    found: List[str] = []
    roots = [
        os.path.expanduser("~/miniconda3/envs/*/bin/python"),
        os.path.expanduser("~/anaconda3/envs/*/bin/python"),
        os.path.expanduser("~/miniforge3/envs/*/bin/python"),
        "/opt/conda/envs/*/bin/python",
        os.path.expanduser("~/.venvs/*/bin/python"),
        os.path.expanduser("~/.virtualenvs/*/bin/python"),
        os.path.expanduser("~/*/.venv/bin/python"),
    ]
    for pat in roots:
        found.extend(glob.glob(pat))
    for p in (sys.executable, shutil.which("python3"), shutil.which("python")):
        if p:
            found.append(p)
    # de-dup by realpath, keep order
    seen, out = set(), []
    for p in found:
        rp = os.path.realpath(p)
        if rp not in seen and os.path.exists(p):
            seen.add(rp)
            out.append(p)
    return out


def _probe_one(py: str, sdk_modules: List[str]) -> Dict:
    """Run a tiny probe inside interpreter `py`: version, SDK import, DC import."""
    code = (
        "import json,sys\n"
        "r={'version':list(sys.version_info[:3]),'sdk':{},'dc':False}\n"
        f"for m in {sdk_modules!r}:\n"
        "    try:\n"
        "        __import__(m); r['sdk'][m]=True\n"
        "    except Exception:\n"
        "        r['sdk'][m]=False\n"
        "try:\n"
        "    import device_connect_edge; r['dc']=True\n"
        "except Exception:\n"
        "    r['dc']=False\n"
        "print(json.dumps(r))\n"
    )
    try:
        out = subprocess.run([py, "-c", code], capture_output=True, text=True, timeout=30)
        data = json.loads(out.stdout.strip().splitlines()[-1])
    except Exception as exc:
        return {"python": py, "error": repr(exc), "version": None, "sdk": {}, "dc": False}
    ver = tuple(data["version"])
    data["python"] = py
    data["version_str"] = ".".join(map(str, ver))
    data["meets_floor"] = ver >= DC_PYTHON_FLOOR
    data["has_sdk"] = any(data["sdk"].values())
    data["env"] = _env_name(py)
    return data


def _env_name(py: str) -> str:
    # .../envs/<name>/bin/python  or  .../<name>/.venv/bin/python
    parts = py.split(os.sep)
    if "envs" in parts:
        i = parts.index("envs")
        if i + 1 < len(parts):
            return parts[i + 1]
    if ".venv" in parts or ".venvs" in parts:
        for tag in (".venv", ".venvs"):
            if tag in parts:
                i = parts.index(tag)
                return parts[i + 1] if tag == ".venvs" and i + 1 < len(parts) else parts[i - 1]
    return os.path.dirname(os.path.dirname(py)).split(os.sep)[-1]


def recommend(probes: List[Dict]) -> Dict:
    sdk_envs = [p for p in probes if p.get("has_sdk")]
    unified = [p for p in sdk_envs if p.get("meets_floor")]
    dc_ready = [p for p in probes if p.get("dc")]
    floor = ".".join(map(str, DC_PYTHON_FLOOR))

    if unified:
        p = unified[0]
        return {
            "mode": "UNIFIED",
            "reason": f"env '{p['env']}' has the SDK AND Python {p['version_str']} (>= {floor}).",
            "actions": [f"{os.path.dirname(p['python'])}/pip install {DC_PACKAGE}"],
            "sdk_env": p["env"], "dc_env": p["env"],
        }
    if sdk_envs:
        sdk = sdk_envs[0]
        existing = next((p for p in dc_ready if p.get("meets_floor")), None)
        dc_env = existing["env"] if existing else "device-connect"
        actions = []
        if not existing:
            actions.append(f"bash bootstrap_dc_env.sh   # create a clean Python>={floor} env "
                           f"'device-connect' and pip install {DC_PACKAGE}")
        actions.append(
            f"Run the DC sidecar in '{dc_env}'; delegate hardware calls (speak/move/read) to the "
            f"SDK env '{sdk['env']}' ({sdk['version_str']}) via subprocess/IPC (see "
            f"references/two-env-bridge.md).")
        return {
            "mode": "BRIDGED",
            "reason": (f"SDK env '{sdk['env']}' is Python {sdk['version_str']} (< {floor}); "
                       f"{DC_PACKAGE} cannot install there. Keep the vendor SDK in its env."),
            "actions": actions, "sdk_env": sdk["env"], "dc_env": dc_env,
        }
    return {
        "mode": "UNKNOWN",
        "reason": "No env with the named SDK module was found. Pass --sdk-module <import_name>.",
        "actions": ["Identify the robot SDK's import name and re-run with --sdk-module.",
                    f"Then bootstrap a clean Python>={floor} env: bash bootstrap_dc_env.sh"],
        "sdk_env": None, "dc_env": "device-connect",
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="Probe Python envs and recommend a Device Connect setup.")
    ap.add_argument("--sdk-module", default=None,
                    help="Comma-separated SDK import name(s) (e.g. unitree_sdk2py). "
                         f"Default tries: {','.join(KNOWN_SDK_MODULES)}")
    ap.add_argument("--json", action="store_true", help="Emit JSON only.")
    args = ap.parse_args()

    sdk_modules = ([m.strip() for m in args.sdk_module.split(",")] if args.sdk_module
                   else list(KNOWN_SDK_MODULES))
    probes = [_probe_one(py, sdk_modules) for py in _find_interpreters()]
    rec = recommend(probes)
    report = {"dc_python_floor": ".".join(map(str, DC_PYTHON_FLOOR)),
              "sdk_modules_probed": sdk_modules, "interpreters": probes, "recommendation": rec}

    if args.json:
        print(json.dumps(report, indent=2))
        return 0

    floor = report["dc_python_floor"]
    print(f"Device Connect requires Python >= {floor}. Probed SDK modules: {', '.join(sdk_modules)}\n")
    print(f"{'ENV':<22} {'PYTHON':<9} {'SDK':<5} {'DC':<4} {'>=floor'}")
    print("-" * 52)
    for p in probes:
        if p.get("version") is None:
            print(f"{p.get('env','?'):<22} {'?':<9} {'-':<5} {'-':<4} (probe failed)")
            continue
        print(f"{p['env']:<22} {p['version_str']:<9} {'yes' if p['has_sdk'] else '-':<5} "
              f"{'yes' if p['dc'] else '-':<4} {'yes' if p['meets_floor'] else 'NO'}")
    print(f"\n>>> RECOMMENDATION: {rec['mode']}")
    print(f"    {rec['reason']}")
    for a in rec["actions"]:
        print(f"    - {a}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
