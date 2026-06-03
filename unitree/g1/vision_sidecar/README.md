# Vision Sidecar

Containerised GPU DINOv2 ViT-S/14 encoder. Runs as a systemd-managed
docker container on the Jetson Orin and exposes a simple length-prefixed
TCP protocol on `127.0.0.1:9878`.

A host-side client encodes an RGB image into a `(384,)` float32 feature
vector, using the sidecar when it is up.

The caller does not need to know whether the encoder lives in-process
(CPU or CUDA) or out-of-process (this sidecar); `get_encoder()` picks
the best available backend at first use.

## Why this exists

DINOv2 ViT-S/14 takes ~250 ms/frame on the Cortex-A78AE cores. The
control loop runs at 10 Hz = 100 ms dt. Even with a background vision
cache that caps the effective vision update rate at ~4 Hz. On the Jetson
Orin GPU the same model lands in
~25-40 ms native CUDA / ~10-15 ms with TensorRT, i.e. fully inside one
control tick with plenty of headroom.

The obvious way to use the GPU — install a JetPack torch wheel into
`unitree_deploy` — would:

- force a python 3.10 → 3.8 downgrade,
- mutate the shared conda env that the other host modules (including
  `depth_camera_sight`) all depend on,
- leave no clean rollback path,
- force every customer install to mirror the same downgrade.

This sidecar isolates the GPU torch wheel inside a container built on
`nvcr.io/nvidia/l4t-pytorch`. The host env is untouched. Rollback is
`./uninstall.sh`.

## Hard constraint: the host env stays pristine

- No new `apt install` or `pip install` into `unitree_deploy`.
- No changes to `/etc/docker/daemon.json`.
- No new kernel modules, drivers, or `/usr/local/cuda-*` changes.
- `nvidia-container-toolkit` mounts `libcuda.so` from the host into the
  container at `docker run --runtime=nvidia` time. That is the entire
  "driver install" — it already happens automatically.

## Files

| File | Purpose |
|---|---|
| `vision_sidecar.py`              | The TCP server — DINOv2 + `socketserver` |
| `Dockerfile`                     | Image recipe: `FROM nvcr.io/nvidia/l4t-pytorch:r35.2.1-pth2.0-py3` + baked weights |
| `robotics-connect-vision-sidecar.service`| systemd unit, `--runtime=nvidia --net=host` |
| `install.sh`                     | Idempotent install: build/load image, install unit, ping |
| `uninstall.sh`                   | Full rollback — stop, remove unit, optional image rm |
| `README.md`                      | This file |

There is no `__init__.py` — matching the layout of the other host
modules, e.g. `depth_camera_sight/`.

## Protocol

Length-prefixed binary framing. Every message — in both directions —
is `struct.pack("<I", length) + body`. This avoids parsing newlines
inside uint8 payloads and handles partial recvs uniformly.

**Ping** (client → server):

```json
{"cmd": "ping"}
```

Response (server → client, JSON-in-envelope):

```json
{"ok": true, "device": "cuda:0", "load_s": 12.3, "torch": "2.0.0+nv23.05"}
```

**Encode** (client → server, two frames):

Frame 1 — JSON header:
```json
{"cmd": "encode", "h": 480, "w": 640, "c": 3}
```

Frame 2 — exactly `h*w*3` raw uint8 RGB bytes.

Response — one frame of 1536 raw little-endian float32 bytes (= 384
floats, the DINOv2 CLS token). On error, response is a JSON blob with
`{"ok": false, "err": "..."}` — the length-prefix always wraps either
shape, so the client inspects byte count first before deciding how to
parse.

## Bring-up (developer flow)

On the robot, with docker + nvidia-container-toolkit already in place
(they are on JetPack 5.1):

```bash
cd /home/unitree/robotics-connect/vision_sidecar
./install.sh
```

This will:

1. Check docker, systemctl, and `nvidia-container-cli` are on PATH.
2. Confirm the nvidia runtime is registered in `docker info`.
3. Build `robotics-connect/vision-sidecar:0.1` from this directory (the
   DINOv2 weights are baked into an image layer so first-run
   cold starts do not need network access).
4. Install `/etc/systemd/system/robotics-connect-vision-sidecar.service`.
5. `systemctl enable --now` it.
6. Ping the sidecar and block until it answers.

Verify it really is on the GPU:

```bash
journalctl -u robotics-connect-vision-sidecar -n 20 --no-pager
# Look for: [vision_sidecar] DINOv2 ViT-S/14 loaded on cuda:0 in ...s
```

Benchmark via the host-side client (inside `unitree_deploy`). A typical
sidecar run reports something like:

```
using sidecar at 127.0.0.1:9878 (remote device=cuda:0 ...)
encode(x50): mean 25.0 ms/frame on cuda:0
```

Compare against the in-process CPU fallback by stopping the sidecar:

```bash
sudo systemctl stop robotics-connect-vision-sidecar
# host-side client now runs in-process: expect ~250 ms/frame on cpu
sudo systemctl start robotics-connect-vision-sidecar
```

The host-side client also exposes an explicit toggle:

```bash
ROBOTICS_CONNECT_VISION_SIDECAR=0    # in-process only
ROBOTICS_CONNECT_VISION_SIDECAR=1    # sidecar-only, no fallback
```

## Health check

```bash
systemctl status robotics-connect-vision-sidecar
# Active: active (running)

python3 -c '
import json, socket, struct
s = socket.create_connection(("127.0.0.1", 9878), timeout=2)
hdr = json.dumps({"cmd":"ping"}).encode()
s.sendall(struct.pack("<I", len(hdr)) + hdr)
(n,) = struct.unpack("<I", s.recv(4))
print(json.loads(s.recv(n)))
'
```

## Upgrading

Bump `IMAGE_TAG` in `install.sh`, the `FROM` line in `Dockerfile`, the
image reference in the systemd unit, and rerun `./install.sh` with
`REBUILD=1`. The unit file re-applies on every install run.

## Rollback

```bash
./uninstall.sh               # stops, disables, removes the unit
./uninstall.sh REMOVE_IMAGE=1  # also drops the image from disk
```

After rollback, the host-side client transparently falls back to the
in-process CPU path — it pings, gets no response, logs
"sidecar unavailable: ..." once, and goes straight to the in-process
encoder. Nothing in the rest of the system notices.

## Stretch: TensorRT engine inside the container

Not wired up yet. The socket protocol does not change — only the
`DinoV2Model.encode` internals swap from `self.model(x)` to a TRT
inference call. Expected to land ~8-12 ms/frame on Orin. Blocked on
needing an ONNX export of DINOv2 ViT-S/14 first. Native CUDA PyTorch is
already fast enough that this is not on any critical path.

## References

- NVIDIA L4T PyTorch containers: <https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-pytorch>
- NVIDIA Container Toolkit: <https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/>
