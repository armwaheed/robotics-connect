# `lib/` — shared internal modules

Small Python modules shared across robotics-connect's skills and robot control stacks, so the same
logic isn't copy-pasted (and allowed to diverge) between, say, `human_agent/` and
`unitree/g1/device_connect/`. Nothing here is a skill or a robot binding — it's the common
scaffolding they sit on.

## `device_connect_sidecar.py`

The one place the Device Connect **sidecar boilerplate** lives, so every device driver shares it
instead of carrying its own copy:

| Export | What it is |
|---|---|
| `HAVE_DC` | is `device-connect-edge` importable in this env? (false → offline self-test mode) |
| `DeviceDriver`, `rpc`, `emit` | re-exported from `device_connect_edge.drivers` (no-SDK stubs as a fallback) |
| `DeviceIdentity`, `DeviceStatus` | re-exported from `device_connect_edge.types` |
| `DEFAULT_NATS_URL` | the hosted fabric (`nats://fabric.deviceconnect.dev:4222`) |
| `load_creds(path)` | read a NATS creds JSON |
| `resolve_urls(creds, override)` | the NATS URLs to register on — strips in-cluster `nats://nats:` hosts, falls back to the public fabric |
| `build_runtime(driver, creds_path, …)` | build (don't run) a `DeviceRuntime` registered on the fabric |

```python
from device_connect_sidecar import HAVE_DC, DeviceDriver, rpc, emit, build_runtime
```

> **Why two drivers share this — the human-agent path.** The bed-making G1 can't hear its human
> partner through its own microphone: Unitree engineering support confirmed (support work order, June
> 2026) that the G1's mic is **not a developer interface** — the only supported options are the
> built-in automatic speech recognition documented under
> [VuiClient_Service](https://support.unitree.com/home/en/G1_developer/VuiClient_Service), or attaching
> an external mic/speaker array over the USB-C ports. So instead of cracking the onboard array, the
> human joins the **Device Connect** fabric as their own agent — a Bluetooth headset plus a sidecar
> that runs local ASR — and the robot asks over its speaker and hears the reply over Device Connect.
> Both sides of that loop (`human_agent/human_agent.py` and `unitree/g1/device_connect/g1_agent.py`)
> are Device Connect drivers built on the boilerplate below — which is exactly why it lives here once.

## Why the cross-reference to Arm's Device Connect

**This module is a thin wrapper, not a reimplementation.** The actual runtime — `DeviceRuntime`, the
`DeviceDriver` base, the `@rpc`/`@emit` decorators, identity/status types, and all the NATS/JWT
registration — comes from Arm's open-source **[device-connect-edge](https://github.com/arm/device-connect)**
(`pip install device-connect-edge`). `device_connect_sidecar.py` only:

1. **re-exports** that API from one import site (so drivers don't each import the SDK and drift), and
2. adds two robotics-connect-specific helpers (`resolve_urls`, `build_runtime`) plus a no-SDK
   import-shim for offline `--self-test`.

So the contract here is **owned upstream**. Two consequences:

- **Pin + track the upstream version.** Built and validated against **`device-connect-edge` 0.2.4**
  (from <https://github.com/arm/device-connect>). If you bump it, re-check `DeviceRuntime(...)` kwargs
  and the `drivers`/`types` import paths the shim mirrors — they are the only coupling points.
- **The stubs are a fallback, not an API.** When `device-connect-edge` is absent the shim provides
  inert stand-ins so a driver still imports (e.g. on a robot's SDK conda env that's pinned to Python
  < 3.11). They are NOT a second implementation to keep in sync with Arm's — on a real fabric the
  genuine SDK is always used.

## Importing it

These are plain modules (not an installed package). Drivers add `lib/` to `sys.path` and import by
name — see `human_agent/human_agent.py` and `unitree/g1/device_connect/g1_agent.py`:

```python
sys.path.insert(0, os.path.join(REPO_ROOT, "lib"))
from device_connect_sidecar import build_runtime
```

> This `sys.path` insertion is the pragmatic choice for run-as-a-script tools; it is the one part of
> this layout that is **not** import-hermetic. If robotics-connect grows more shared code, promote
> this to an installable package (`pyproject.toml` + `pip install -e .`) so imports are real and
> path-surgery-free.
