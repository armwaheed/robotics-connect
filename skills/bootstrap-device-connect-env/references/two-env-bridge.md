# The two-env bridge — run Device Connect next to a vendor SDK that pins an older Python

## When you need it

`device-connect-edge` requires **Python ≥ 3.11**. Most humanoid vendor SDKs pin an **older** Python
and carry native deps that are painful to rebuild on a newer interpreter:

| Robot / stack | SDK Python | Why you can't just upgrade |
|---|---|---|
| Unitree G1 / Go2 (`unitree_sdk2py`) | 3.10 (often 3.8) | CycloneDDS Cython bindings built for that Python |
| ROS 2 Humble (`rclpy`) | 3.10 | the whole ROS install is built against 3.10 |
| Jetson vendor stacks | 3.8 / 3.10 | vendor wheels (TensorRT, etc.) pinned to that Python |

Forcing the SDK onto 3.11 (rebuilding CycloneDDS, re-pinning vendor wheels) is how you brick a
working robot. **Don't.** Keep the vendor SDK in its env; run Device Connect in its own ≥3.11 env;
bridge between them.

> This is not a workaround — it's the correct ownership boundary. The vendor owns their env and its
> Python; you own the Device Connect env and its Python. A subprocess/IPC call across the boundary
> is cheap and keeps both installs intact.

## The pattern

```
┌────────────────────────────┐         subprocess / local socket         ┌───────────────────────────┐
│  Device Connect sidecar     │  ──────────────────────────────────────▶ │  Vendor SDK env (pinned)   │
│  env: device-connect (3.11) │   "speak this" / "move to X" / "read Y"   │  env: <sdk> (3.10/3.8)     │
│  device-connect-edge        │ ◀──────────────────────────────────────  │  unitree_sdk2py + DDS      │
│  @rpc say / request_help    │                 result                    │  drives the hardware       │
└────────────────────────────┘                                           └───────────────────────────┘
        │ registers on the NATS fabric (dashboard) + invoke_remote() to other DC devices
```

- The **DC sidecar** (≥3.11 env) registers the robot on the fabric, exposes its `@rpc` functions,
  and talks to other devices over Device Connect (`invoke_remote`).
- Each hardware action delegates to a **small CLI in the SDK env**, run as a subprocess. The SDK
  env is found once (its interpreter path) and reused.
- A fresh subprocess re-inits DDS each call (~1–2 s) — fine for low-frequency actions (speech,
  goals). For high-frequency control, replace the subprocess with a **long-running SDK daemon** the
  sidecar talks to over a localhost socket (same boundary, lower latency).

## Worked example — the Unitree G1 EDU

The G1 EDU's SDK env is `robotics-connect` / `unitree_deploy` (Python 3.10, `unitree_sdk2py` +
CycloneDDS). Device Connect runs in `dc-repro` (Python 3.11). The sidecar speaks through the robot's
chest speaker by delegating to the SDK env. Simplified shape (see the real
`unitree/g1/device_connect/g1_agent.py` for the full version — it uses absolute script paths,
captures output, and checks the exit code):

```python
# g1_agent.py  (runs in the 3.11 Device Connect env) — illustrative
SDK_PY = "/home/unitree/miniconda3/envs/robotics-connect/bin/python"

def speak_blocking(text):                      # delegate to the SDK env (returns True on exit 0)
    env = {**os.environ, "G1_DDS_IFACE": "eth0"}
    return subprocess.run([SDK_PY, SPEAK_SCRIPT, text], env=env).returncode == 0

class G1AgentDriver(DeviceDriver):
    device_type = "unitree_g1"
    @rpc()
    async def request_help(self, question, choices="yesno"):
        await asyncio.get_event_loop().run_in_executor(None, speak_blocking, question)  # speaker
        return await self.invoke_remote("beta-...-human-agent", "ask",    # human answers over DC
                                        question=question, choices=choices, prompt_earpiece=False)
```

```python
# g1_speak.py  (runs in the SDK env — unitree_sdk2py + CycloneDDS on eth0)
from g1_voice import G1Speaker
G1Speaker(iface="eth0", default_volume=100).say(sys.argv[1])
```

Find the SDK interpreter and confirm the split with `probe_dc_env.py --sdk-module unitree_sdk2py`.
