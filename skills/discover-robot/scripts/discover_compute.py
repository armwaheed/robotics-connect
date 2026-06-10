#!/usr/bin/env python3
"""Discover a robot's compute topology into the descriptor's `compute` block (onboard + peripheral).

The "any humanoid" theme applies to GPUs: a robot may have 0, 1, or N accelerators, onboard or on an
expansion port (e.g. the Unitree G1's rear port for an NVIDIA Jetson Thor — a SEPARATE compute node, not
a `cuda:N` on the onboard SoC). This enumerates what's actually present and emits the `compute` array
that the vision sidecar (and any GPU workload) is placed + targeted from.

Onboard enumeration tries, in order (best-effort, vendor-neutral where possible):
  1. local ``torch.cuda`` (if a CUDA-enabled torch is importable here),
  2. a GPU container's CUDA runtime — a Jetson host's torch is usually CPU-only, but the vision-sidecar
     image has working CUDA, so we enumerate inside it,
  3. ``nvidia-smi``.

Peripheral nodes: pass ``--expansion host[:port],...`` (each an expansion node running a sidecar); we
ping ``host:9878`` for its topology. A declared-but-unreachable expansion node is emitted with
``present: false`` so a stale slot is visible rather than silently dropped.

Usage:
  python discover_compute.py                                   # onboard only
  python discover_compute.py --image robotics-connect/vision-sidecar:0.1
  python discover_compute.py --expansion 192.168.123.50        # + a peripheral Thor's sidecar
"""

from __future__ import annotations

import argparse
import json
import shutil
import socket
import struct
import subprocess


def _onboard_via_torch():
    try:
        import torch
    except Exception:
        return None
    if not torch.cuda.is_available():
        return None
    out = []
    for i in range(torch.cuda.device_count()):
        p = torch.cuda.get_device_properties(i)
        out.append({"index": i, "name": p.name, "memory_gb": round(p.total_memory / (1024 ** 3), 1)})
    return out


def _onboard_via_container(image: str | None):
    """Enumerate CUDA devices inside a GPU container (for hosts whose own torch is CPU-only)."""
    if not shutil.which("docker"):
        return None
    if not image:
        try:
            imgs = subprocess.run(["docker", "images", "--format", "{{.Repository}}:{{.Tag}}"],
                                  capture_output=True, text=True, timeout=15).stdout.split()
        except Exception:
            return None
        image = next((i for i in imgs if "vision-sidecar" in i), None)
    if not image:
        return None
    code = ("import torch,json;"
            "d=[{'index':i,'name':torch.cuda.get_device_properties(i).name,"
            "'memory_gb':round(torch.cuda.get_device_properties(i).total_memory/(1024**3),1)}"
            "for i in range(torch.cuda.device_count())] if torch.cuda.is_available() else [];"
            "print(json.dumps(d))")
    try:
        r = subprocess.run(["docker", "run", "--rm", "--runtime=nvidia", "--entrypoint", "python3",
                            image, "-c", code], capture_output=True, text=True, timeout=120)
        return json.loads(r.stdout.strip().splitlines()[-1]) or None
    except Exception:
        return None


def _onboard_via_nvidia_smi():
    if not shutil.which("nvidia-smi"):
        return None
    try:
        r = subprocess.run(["nvidia-smi", "--query-gpu=name,memory.total", "--format=csv,noheader"],
                           capture_output=True, text=True, timeout=15)
    except Exception:
        return None
    out = []
    for i, line in enumerate(r.stdout.strip().splitlines()):
        name, mem = (p.strip() for p in line.split(",", 1))
        mb = float(mem.split()[0]) if mem.split() else 0.0
        out.append({"index": i, "name": name, "memory_gb": round(mb / 1024, 1)})
    return out or None


def _classify(name: str) -> str:
    n = name.lower()
    if any(k in n for k in ("jetson", "orin", "xavier", "tegra", "thor", "igpu")):
        return "igpu"
    return "dgpu"


def _ping_sidecar(host: str, port: int, timeout: float = 4.0):
    """Ping a sidecar on a (peripheral) node and return its topology, or None if unreachable."""
    try:
        s = socket.create_connection((host, port), timeout=timeout)
        hdr = json.dumps({"cmd": "ping"}).encode()
        s.sendall(struct.pack("<I", len(hdr)) + hdr)
        (n,) = struct.unpack("<I", _recvall(s, 4))
        info = json.loads(_recvall(s, n).decode())
        s.close()
        return info
    except Exception:
        return None


def _recvall(s, n):
    buf = b""
    while len(buf) < n:
        chunk = s.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("short read")
        buf += chunk
    return buf


def discover(image: str | None, expansion: list[str]) -> list[dict]:
    compute = []

    # ── onboard ──────────────────────────────────────────────────────────────
    devs = _onboard_via_torch() or _onboard_via_container(image) or _onboard_via_nvidia_smi()
    if devs:
        for d in devs:
            compute.append({
                "node": "onboard" if d["index"] == 0 else f"onboard-{d['index']}",
                "location": "onboard",
                "host": "127.0.0.1",
                "type": _classify(d["name"]),
                "model": d["name"],
                "device": f"cuda:{d['index']}",
                "memory_gb": d["memory_gb"],
                "framework": "cuda",
                "present": True,
            })
    else:
        compute.append({"node": "onboard", "location": "onboard", "host": "127.0.0.1",
                        "type": "cpu", "model": "(no CUDA accelerator found)", "framework": "cpu",
                        "present": True})

    # ── peripheral / expansion nodes ─────────────────────────────────────────
    for k, spec in enumerate(expansion):
        host, _, p = spec.partition(":")
        port = int(p) if p else 9878
        info = _ping_sidecar(host, port)
        node = {"node": f"expansion-{k}", "location": "expansion", "host": host,
                "type": "igpu", "framework": "cuda", "optional": True}
        if info and info.get("ok"):
            node["present"] = True
            devs = info.get("devices") or []
            if devs:
                node["model"] = devs[0]["name"]
                node["memory_gb"] = devs[0]["memory_gb"]
            node["notes"] = f"sidecar reachable; device_count={info.get('device_count')}"
        else:
            node["present"] = False
            node["notes"] = "declared expansion node not reachable (slot empty or sidecar down)"
        compute.append(node)

    return compute


def main():
    ap = argparse.ArgumentParser(description="Discover the robot's compute/GPU topology.")
    ap.add_argument("--image", default=None, help="GPU container image to enumerate inside (auto-detected if omitted).")
    ap.add_argument("--expansion", default="", help="Comma-separated expansion node hosts (each host[:port]).")
    args = ap.parse_args()
    expansion = [s for s in args.expansion.split(",") if s.strip()]
    print(json.dumps(discover(args.image, expansion), indent=2))


if __name__ == "__main__":
    main()
