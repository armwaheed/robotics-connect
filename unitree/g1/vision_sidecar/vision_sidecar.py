#!/usr/bin/env python3
"""
Vision sidecar — GPU DINOv2 ViT-S/14 encoder exposed over a local TCP socket.

Runs inside a `nvcr.io/nvidia/l4t-pytorch` container with `--runtime=nvidia`
so that `torch.cuda.is_available()` is True even though the host
`unitree_deploy` conda env ships CPU-only torch.

PROTOCOL
--------
Length-prefixed binary framing on `127.0.0.1:9878` (one client at a time
is fine — the caller is a single background thread).

  client → server:   struct.pack("<I", header_len) + header_json_utf8
                     followed by `payload_bytes` raw uint8 bytes
                     (payload_bytes is inferred from the header).

  server → client:   struct.pack("<I", response_len) + response_bytes

Headers are JSON:

  {"cmd": "ping"}
    → response is a JSON line: {"ok": true, "device": "...",
                                 "load_s": 12.3, "torch": "2.0.0+nv23.05"}
    payload_bytes == 0

  {"cmd": "encode", "h": H, "w": W, "c": 3}
    → payload is H*W*3 uint8 RGB bytes.
    → response is 384 * 4 = 1536 raw little-endian float32 bytes (no JSON).

Any other cmd, or a malformed header, returns a JSON error response
(same length-prefixed envelope) and the connection is closed.

The server logs one line per `encode` call with wall-clock ms so the
operator can watch actual GPU latency at steady state.
"""
from __future__ import annotations

import json
import os
import socket
import socketserver
import struct
import sys
import threading
import time
import traceback
from typing import Optional

import numpy as np
import torch
import torch.nn.functional as F


DINOV2_FEAT_DIM = 384
DINOV2_INPUT = 224

_IMAGENET_MEAN = (0.485, 0.456, 0.406)
_IMAGENET_STD  = (0.229, 0.224, 0.225)


class DinoV2Model:
    """One-time DINOv2 ViT-S/14 load. Matches the host encoder contract.

    Kept intentionally similar to the host-side encoder so that outputs
    are numerically identical modulo CPU/GPU floating point.
    """

    def __init__(self, device: str):
        self.device = torch.device(device)
        t0 = time.monotonic()
        # DINOv2 is cloned into the image at a Python 3.8-compatible
        # commit — see Dockerfile for the rationale. We load from the
        # local checkout because torch.hub.load("user/repo:<sha>") only
        # supports branches/tags, not raw commit hashes.
        dinov2_local = os.environ.get("DINOV2_LOCAL", "/opt/dinov2")
        self.model = torch.hub.load(
            dinov2_local,
            "dinov2_vits14",
            source="local",
            trust_repo=True,
            verbose=False,
        )
        self.model.eval()
        for p in self.model.parameters():
            p.requires_grad_(False)
        self.model.to(self.device)

        self._mean = torch.tensor(_IMAGENET_MEAN, device=self.device).view(1, 3, 1, 1)
        self._std  = torch.tensor(_IMAGENET_STD,  device=self.device).view(1, 3, 1, 1)

        with torch.no_grad():
            dummy = torch.zeros(1, 3, DINOV2_INPUT, DINOV2_INPUT, device=self.device)
            self.model(dummy)
            if self.device.type == "cuda":
                torch.cuda.synchronize()

        self.load_s = time.monotonic() - t0

    def encode(self, rgb: np.ndarray) -> np.ndarray:
        with torch.no_grad():
            t = torch.from_numpy(rgb).to(self.device)
            t = t.permute(2, 0, 1).unsqueeze(0).float().div_(255.0)
            t = F.interpolate(t, size=(DINOV2_INPUT, DINOV2_INPUT),
                              mode="bilinear", align_corners=False)
            t = (t - self._mean) / self._std
            feat = self.model(t)
            if self.device.type == "cuda":
                torch.cuda.synchronize()
            return feat[0].detach().to("cpu").numpy().astype(np.float32)


# Module singleton — loaded once at process start, shared across requests.
_MODEL: Optional[DinoV2Model] = None
_MODEL_LOCK = threading.Lock()


def _get_model() -> DinoV2Model:
    global _MODEL
    with _MODEL_LOCK:
        if _MODEL is None:
            device = "cuda" if torch.cuda.is_available() else "cpu"
            _MODEL = DinoV2Model(device=device)
            print(f"[vision_sidecar] DINOv2 ViT-S/14 loaded on {_MODEL.device} "
                  f"in {_MODEL.load_s:.2f}s  torch={torch.__version__}",
                  flush=True)
        return _MODEL


# ── Framing helpers ──────────────────────────────────────────────────────────

def _recvall(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(min(65536, n - len(buf)))
        if not chunk:
            raise ConnectionError(f"short read: expected {n}, got {len(buf)}")
        buf.extend(chunk)
    return bytes(buf)


def _read_frame(sock: socket.socket) -> bytes:
    hdr = _recvall(sock, 4)
    (n,) = struct.unpack("<I", hdr)
    if n == 0:
        return b""
    if n > 64 * 1024 * 1024:
        raise ValueError(f"frame too large: {n} bytes")
    return _recvall(sock, n)


def _send_frame(sock: socket.socket, payload: bytes) -> None:
    sock.sendall(struct.pack("<I", len(payload)) + payload)


def _send_json(sock: socket.socket, obj) -> None:
    _send_frame(sock, json.dumps(obj).encode("utf-8"))


# ── Handler ──────────────────────────────────────────────────────────────────

class _Handler(socketserver.BaseRequestHandler):
    def handle(self):
        sock = self.request
        try:
            header_bytes = _read_frame(sock)
            try:
                header = json.loads(header_bytes.decode("utf-8"))
            except Exception as e:  # noqa: BLE001
                _send_json(sock, {"ok": False, "err": f"bad header: {e!r}"})
                return

            cmd = header.get("cmd")
            if cmd == "ping":
                model = _get_model()
                _send_json(sock, {
                    "ok": True,
                    "device": str(model.device),
                    "load_s": round(model.load_s, 3),
                    "torch": torch.__version__,
                    "cuda_available": bool(torch.cuda.is_available()),
                })
                return

            if cmd == "encode":
                h = int(header["h"])
                w = int(header["w"])
                c = int(header.get("c", 3))
                if c != 3:
                    _send_json(sock, {"ok": False, "err": f"c must be 3, got {c}"})
                    return
                expected = h * w * c
                payload = _read_frame(sock)
                if len(payload) != expected:
                    _send_json(sock, {
                        "ok": False,
                        "err": f"payload size {len(payload)} != h*w*c={expected}",
                    })
                    return
                rgb = np.frombuffer(payload, dtype=np.uint8).reshape(h, w, c)
                model = _get_model()
                t0 = time.monotonic()
                feat = model.encode(rgb)
                dt_ms = (time.monotonic() - t0) * 1000.0
                out = feat.astype("<f4", copy=False).tobytes()
                if len(out) != DINOV2_FEAT_DIM * 4:
                    _send_json(sock, {
                        "ok": False,
                        "err": f"bad feature length {len(out)}",
                    })
                    return
                _send_frame(sock, out)
                print(f"[vision_sidecar] encode {h}x{w}  {dt_ms:6.1f} ms  "
                      f"device={model.device}", flush=True)
                return

            _send_json(sock, {"ok": False, "err": f"unknown cmd: {cmd!r}"})

        except ConnectionError as e:
            print(f"[vision_sidecar] connection error: {e}", flush=True)
        except Exception as e:  # noqa: BLE001
            traceback.print_exc()
            try:
                _send_json(sock, {"ok": False, "err": f"{type(e).__name__}: {e}"})
            except Exception:  # noqa: BLE001
                pass


class _ReusableServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main() -> int:
    host = os.environ.get("VISION_SIDECAR_HOST", "0.0.0.0")
    port = int(os.environ.get("VISION_SIDECAR_PORT", "9878"))

    # Warm the model eagerly so the first client ping reflects real state.
    _get_model()

    srv = _ReusableServer((host, port), _Handler)
    print(f"[vision_sidecar] listening on {host}:{port}", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("[vision_sidecar] shutting down", flush=True)
        srv.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
