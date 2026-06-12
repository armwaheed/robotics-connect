#!/usr/bin/env python3
"""Rabia — the physical Unitree G1 EDU, as a Device Connect device.

Registers the real G1 EDU ("Rabia") on the Arm Device Connect fabric so it appears live on the
dashboard next to the Human Agent, and exposes the help-seeking loop for bed-making
(armwaheed/robots#3):

  say(text)                         -> speak through Rabia's OWN chest speaker (the OUT-LOUD channel)
  request_help(question, choices)   -> speak the question aloud AND ask the Human Agent over Device
                                       Connect; return the human's grounded answer
  get_status()                      -> robot status

Events (visible in the dashboard event stream):
  help_requested(question, choices) ; help_answered(question, choice, transcript)

ARCHITECTURE — why this delegates speaking to a subprocess:
  device-connect-edge needs Python >=3.11; the Unitree SDK env (unitree_sdk2py + CycloneDDS) on the
  robot is 3.10. So this sidecar runs in the 3.11 env and speaks by invoking ``rabia_speak.py`` in
  the SDK env (same robot, DDS on eth0). DC is the source of truth for the exchange; the chest
  speaker is the human-facing rendering of it. The human ANSWERS through the headset mic (the Human
  Agent), so request_help() asks with ``prompt_earpiece=False`` — the robot speaker carries the
  question, per the agreed out-loud design.

Run ON the robot (3.11 env):
  python rabia_agent.py --creds /path/to/beta-rabia-waheeds-unitree-g1.creds.json
Speaker-only smoke test (no fabric):
  python rabia_agent.py --say "Hello, I am Rabia."
"""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import os
import subprocess
import sys

log = logging.getLogger("rabia")

# Defaults for delegating speech to the SDK env on the robot.
DEFAULT_SPEAK_PY = "/home/unitree/miniconda3/envs/robotics-connect/bin/python"
DEFAULT_SPEAK_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rabia_speak.py")
DEFAULT_HUMAN_AGENT = "beta-bluetooth-headset-human-agent"
DEFAULT_NATS_URL = "nats://fabric.deviceconnect.dev:4222"

try:
    from device_connect_edge.drivers import DeviceDriver, rpc, emit
    from device_connect_edge.types import DeviceIdentity, DeviceStatus
    _HAVE_DC = True
except Exception:
    _HAVE_DC = False
    DeviceDriver = object  # type: ignore

    def rpc(*a, **k):  # type: ignore
        def deco(fn):
            return fn
        return deco

    emit = rpc  # type: ignore


def speak_blocking(text: str, speak_py: str, speak_script: str, dds_iface: str,
                   timeout: float = 30.0) -> bool:
    """Speak `text` via the chest speaker by running rabia_speak.py in the SDK env."""
    env = dict(os.environ, RABIA_DDS_IFACE=dds_iface)
    try:
        r = subprocess.run([speak_py, speak_script, text], env=env,
                           capture_output=True, text=True, timeout=timeout)
        # rabia_speak.py exits 0 only when the chest speaker actually spoke (TtsMaker ok);
        # use the exit code, not stdout (g1_voice prints its own lines to stdout).
        if r.returncode != 0:
            log.warning("rabia_speak rc=%s out=%r err=%r", r.returncode,
                        (r.stdout or "").strip(), (r.stderr or "")[-300:])
        return r.returncode == 0
    except Exception as exc:
        log.warning("speak failed: %r", exc)
        return False


class RabiaDriver(DeviceDriver):
    """The Unitree G1 EDU 'Rabia' as a Device Connect device."""

    device_type = "unitree_g1"

    def __init__(self, name: str = "Rabia", human_agent_id: str = DEFAULT_HUMAN_AGENT,
                 speak_py: str = DEFAULT_SPEAK_PY, speak_script: str = DEFAULT_SPEAK_SCRIPT,
                 dds_iface: str = "eth0"):
        if _HAVE_DC:
            super().__init__()
        self.name = name
        self.human_agent_id = human_agent_id
        self.speak_py = speak_py
        self.speak_script = speak_script
        self.dds_iface = dds_iface

    @property
    def identity(self) -> "DeviceIdentity":
        return DeviceIdentity(
            device_type="unitree_g1",
            manufacturer="Unitree",
            model=f"G1 EDU ({self.name})",
            description=(f"{self.name} — a physical 23-DOF Unitree G1 EDU humanoid making a bed with "
                         f"a human partner. Makes the bed solo and asks the human for help (out loud, "
                         f"through its chest speaker) when stuck, over Device Connect."),
        )

    @property
    def status(self) -> "DeviceStatus":
        return DeviceStatus(availability="available")

    @emit()
    async def help_requested(self, question: str, choices):
        """Rabia asked its human partner for help."""

    @emit()
    async def help_answered(self, question: str, choice, transcript: str):
        """The human partner answered (grounded)."""

    @rpc()
    async def say(self, text: str) -> dict:
        """Speak `text` aloud through Rabia's chest speaker (the out-loud channel)."""
        loop = asyncio.get_event_loop()
        spoken = await loop.run_in_executor(
            None, speak_blocking, text, self.speak_py, self.speak_script, self.dds_iface)
        return {"spoken": bool(spoken), "text": text}

    @rpc()
    async def request_help(self, question: str, choices="yesno", listen_seconds: float = 8.0) -> dict:
        """Ask the human partner for help: speak the question ALOUD, then capture the spoken answer
        via the Human Agent over Device Connect, and return the grounded decision.

        The robot's chest speaker carries the question (out-loud); the human answers through the
        headset mic (the Human Agent asks with prompt_earpiece=False so it doesn't double-speak)."""
        loop = asyncio.get_event_loop()
        await self.help_requested(question=question, choices=choices)
        # 1) Speak the question aloud (blocks for the spoken duration), THEN listen.
        spoken = await loop.run_in_executor(
            None, speak_blocking, question, self.speak_py, self.speak_script, self.dds_iface)
        # 2) Consult the Human Agent over Device Connect.
        answer = {"heard": False, "choice": None, "transcript": "", "error": None}
        try:
            resp = await self.invoke_remote(
                self.human_agent_id, "ask", timeout=listen_seconds + 15.0,
                question=question, choices=choices, listen_seconds=listen_seconds,
                prompt_earpiece=False)
            answer = resp.get("result", resp) if isinstance(resp, dict) else resp
            if isinstance(resp, dict) and resp.get("error"):
                answer = {"heard": False, "choice": None, "transcript": "",
                          "error": str(resp["error"])}
        except Exception as exc:
            log.warning("request_help: consulting human agent failed: %r", exc)
            answer = {"heard": False, "choice": None, "transcript": "", "error": str(exc)}
        await self.help_answered(question=question, choice=answer.get("choice"),
                                 transcript=answer.get("transcript", ""))
        answer["spoke_question"] = bool(spoken)
        return answer

    @rpc()
    async def get_status(self) -> dict:
        return {"name": self.name, "device_type": self.device_type, "dds_iface": self.dds_iface,
                "human_agent": self.human_agent_id, "availability": "available"}


# ──────────────────────────────────────────────────────────────────────────────────────────────
def _load_creds(path: str) -> dict:
    with open(os.path.expanduser(path)) as f:
        return json.load(f)


def _resolve_urls(creds: dict, override: str | None) -> list:
    if override:
        return [override]
    urls = creds.get("nats", {}).get("urls") or []
    # In-cluster hostnames (nats://nats:4222) aren't routable off the cluster — use the public fabric.
    routable = [u for u in urls if "://nats:" not in u and "://nats/" not in u]
    return routable or [DEFAULT_NATS_URL]


async def _run_fabric(args) -> None:
    if not _HAVE_DC:
        raise RuntimeError("device-connect-edge not installed in this env.")
    from device_connect_edge import DeviceRuntime

    creds = _load_creds(args.creds)
    device_id = args.device_id or creds.get("device_id") or "rabia"
    urls = _resolve_urls(creds, args.nats_url)
    driver = RabiaDriver(name=args.name, human_agent_id=args.human_agent_id,
                         speak_py=args.speak_python, speak_script=args.speak_script,
                         dds_iface=args.dds_iface)
    log.info("Rabia '%s' registering on %s (tenant=%s); speaker via %s",
             device_id, urls, creds.get("tenant", "default"), args.speak_python)
    rt = DeviceRuntime(driver=driver, device_id=device_id, messaging_backend="nats",
                       messaging_urls=urls, credentials_file=os.path.expanduser(args.creds),
                       tenant=creds.get("tenant", "default"))
    if args.demo_help:
        # One-shot: after registering, Rabia asks her human partner for help (the real trigger is
        # the competence monitor; this exercises the same path on demand).
        rt_task = asyncio.create_task(rt.run())
        await asyncio.sleep(args.demo_delay)
        log.info("DEMO: Rabia asks for help — %r", args.demo_help)
        res = await driver.request_help(args.demo_help)
        log.info("DEMO request_help result: %s", json.dumps(res))
        await rt_task
    else:
        await rt.run()


def main() -> None:
    ap = argparse.ArgumentParser(description="Rabia — the physical Unitree G1 EDU as a Device "
                                             "Connect device.")
    ap.add_argument("--creds", help="NATS creds JSON (registers Rabia on the dashboard fabric).")
    ap.add_argument("--device-id", default=None, help="Override device id (default: from creds).")
    ap.add_argument("--nats-url", default=None, help="Override NATS url.")
    ap.add_argument("--name", default="Rabia", help="Robot display name.")
    ap.add_argument("--human-agent-id", default=DEFAULT_HUMAN_AGENT, help="Human Agent device id to consult.")
    ap.add_argument("--speak-python", default=DEFAULT_SPEAK_PY, help="Python in the SDK env (for the chest speaker).")
    ap.add_argument("--speak-script", default=DEFAULT_SPEAK_SCRIPT, help="Path to rabia_speak.py.")
    ap.add_argument("--dds-iface", default=os.environ.get("RABIA_DDS_IFACE", "eth0"), help="DDS interface to the robot.")
    ap.add_argument("--say", default=None, help="Speaker-only smoke test: speak this text and exit (no fabric).")
    ap.add_argument("--demo-help", default=None,
                    help="After registering, ask the human this question once (full out-loud loop).")
    ap.add_argument("--demo-delay", type=float, default=6.0, help="Seconds to wait before --demo-help.")
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s  %(name)-8s  %(levelname)-7s  %(message)s")

    if args.say is not None:
        ok = speak_blocking(args.say, args.speak_python, args.speak_script, args.dds_iface)
        print("spoke" if ok else "FAILED")
        return
    if not args.creds:
        ap.error("--creds is required (or use --say for a speaker-only test).")
    try:
        asyncio.run(_run_fabric(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
