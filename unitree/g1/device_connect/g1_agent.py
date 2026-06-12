#!/usr/bin/env python3
"""The Unitree G1 EDU as a Device Connect device.

Registers the physical G1 EDU on the Arm Device Connect fabric so it appears live on the dashboard
next to a Human Agent, and exposes the help-seeking loop for human-in-the-loop tasks (e.g. the
bed-making demo in the armwaheed/robots repo):

  say(text)                         -> speak through the G1's OWN chest speaker (the OUT-LOUD channel)
  request_help(question, choices)   -> speak the question aloud AND ask the Human Agent over Device
                                       Connect; return the human's grounded answer
  get_status()                      -> robot status

Events (visible in the dashboard event stream):
  help_requested(question, choices) ; help_answered(question, choice, transcript)

ARCHITECTURE — why this delegates speaking to a subprocess:
  device-connect-edge needs Python >=3.11; the Unitree SDK env (unitree_sdk2py + CycloneDDS) on the
  robot is 3.10. So this sidecar runs in the 3.11 env and speaks by invoking ``g1_speak.py`` in the
  SDK env (same robot, DDS on eth0) — the two-env bridge (see the bootstrap-device-connect-env
  skill). Device Connect is the source of truth for the exchange; the chest speaker is the
  human-facing rendering of it. The human ANSWERS through the headset mic (the Human Agent), so
  request_help() asks with ``prompt_earpiece=False`` — the robot speaker carries the question.

Run ON the robot (3.11 env):
  python g1_agent.py --creds /path/to/<robot>.creds.json [--name "..."]
Speaker-only smoke test (no fabric):
  python g1_agent.py --say "Hello."
"""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import os
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "lib"))  # for device_connect_sidecar
from device_connect_sidecar import (  # noqa: E402
    HAVE_DC, DeviceDriver, rpc, emit, DeviceIdentity, DeviceStatus, build_runtime, DEFAULT_NATS_URL,
)

log = logging.getLogger("g1-agent")

# Defaults for delegating speech to the SDK env on the robot.
DEFAULT_SPEAK_PY = "/home/unitree/miniconda3/envs/robotics-connect/bin/python"
DEFAULT_SPEAK_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "g1_speak.py")
DEFAULT_HUMAN_AGENT = "beta-bluetooth-headset-human-agent"


def speak_blocking(text: str, speak_py: str, speak_script: str, dds_iface: str,
                   timeout: float = 30.0) -> bool:
    """Speak `text` via the chest speaker by running g1_speak.py in the SDK env."""
    env = dict(os.environ, G1_DDS_IFACE=dds_iface)
    try:
        r = subprocess.run([speak_py, speak_script, text], env=env,
                           capture_output=True, text=True, timeout=timeout)
        # g1_speak.py exits 0 only when the chest speaker actually spoke (TtsMaker ok);
        # use the exit code, not stdout (g1_voice prints its own lines to stdout).
        if r.returncode != 0:
            log.warning("g1_speak rc=%s out=%r err=%r", r.returncode,
                        (r.stdout or "").strip(), (r.stderr or "")[-300:])
        return r.returncode == 0
    except Exception as exc:
        log.warning("speak failed: %r", exc)
        return False


class G1AgentDriver(DeviceDriver):
    """The Unitree G1 EDU as a Device Connect device."""

    device_type = "unitree_g1"

    def __init__(self, name: str = "G1 EDU", human_agent_id: str = DEFAULT_HUMAN_AGENT,
                 speak_py: str = DEFAULT_SPEAK_PY, speak_script: str = DEFAULT_SPEAK_SCRIPT,
                 dds_iface: str = "eth0"):
        if HAVE_DC:
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
            model=f"G1 EDU ({self.name})" if self.name and self.name != "G1 EDU" else "G1 EDU",
            description=(f"{self.name} — a physical 23-DOF Unitree G1 EDU humanoid working with a "
                         f"human partner. Makes the bed solo and asks the human for help (out loud, "
                         f"through its chest speaker) when stuck, over Device Connect."),
        )

    @property
    def status(self) -> "DeviceStatus":
        return DeviceStatus(availability="available")

    @emit()
    async def help_requested(self, question: str, choices):
        """The robot asked its human partner for help."""

    @emit()
    async def help_answered(self, question: str, choice, transcript: str):
        """The human partner answered (grounded)."""

    @rpc()
    async def say(self, text: str) -> dict:
        """Speak `text` aloud through the G1's chest speaker (the out-loud channel)."""
        loop = asyncio.get_running_loop()
        spoken = await loop.run_in_executor(
            None, speak_blocking, text, self.speak_py, self.speak_script, self.dds_iface)
        return {"spoken": bool(spoken), "text": text}

    @rpc()
    async def request_help(self, question: str, choices="yesno", listen_seconds: float = 8.0) -> dict:
        """Ask the human partner for help: speak the question ALOUD, then capture the spoken answer
        via the Human Agent over Device Connect, and return the grounded decision.

        The chest speaker carries the question (out loud); the human answers through the headset mic
        (the Human Agent asks with prompt_earpiece=False so it doesn't double-speak)."""
        loop = asyncio.get_running_loop()
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
            result = resp.get("result", resp) if isinstance(resp, dict) else resp
            if isinstance(resp, dict) and resp.get("error"):
                answer = {"heard": False, "choice": None, "transcript": "", "error": str(resp["error"])}
            elif isinstance(result, dict):
                answer = result
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
async def _run_fabric(args) -> None:
    driver = G1AgentDriver(name=args.name, human_agent_id=args.human_agent_id,
                           speak_py=args.speak_python, speak_script=args.speak_script,
                           dds_iface=args.dds_iface)
    rt, creds, urls = build_runtime(driver, args.creds, device_id=args.device_id,
                                    nats_url=args.nats_url)
    log.info("G1 agent '%s' (%s) registering on %s (tenant=%s); speaker via %s",
             creds.get("device_id"), args.name, urls, creds.get("tenant", "default"), args.speak_python)
    if args.demo_help:
        # One-shot: after registering, the robot asks its human partner for help (the real trigger
        # is the competence monitor; this exercises the same path on demand).
        rt_task = asyncio.create_task(rt.run())
        await asyncio.sleep(args.demo_delay)
        log.info("DEMO: the robot asks for help — %r", args.demo_help)
        res = await driver.request_help(args.demo_help)
        log.info("DEMO request_help result: %s", json.dumps(res))
        await rt_task
    else:
        await rt.run()


def main() -> None:
    ap = argparse.ArgumentParser(description="The Unitree G1 EDU as a Device Connect device.")
    ap.add_argument("--creds", help="NATS creds JSON (registers the robot on the dashboard fabric).")
    ap.add_argument("--device-id", default=None, help="Override device id (default: from creds).")
    ap.add_argument("--nats-url", default=None, help="Override NATS url.")
    ap.add_argument("--name", default=os.environ.get("G1_AGENT_NAME", "G1 EDU"),
                    help="Robot display name (the bed-making demo passes its own robot's name).")
    ap.add_argument("--human-agent-id", default=DEFAULT_HUMAN_AGENT, help="Human Agent device id to consult.")
    ap.add_argument("--speak-python", default=DEFAULT_SPEAK_PY, help="Python in the SDK env (for the chest speaker).")
    ap.add_argument("--speak-script", default=DEFAULT_SPEAK_SCRIPT, help="Path to g1_speak.py.")
    ap.add_argument("--dds-iface", default=os.environ.get("G1_DDS_IFACE", "eth0"), help="DDS interface to the robot.")
    ap.add_argument("--say", default=None, help="Speaker-only smoke test: speak this text and exit (no fabric).")
    ap.add_argument("--demo-help", default=None,
                    help="After registering, ask the human this question once (full out-loud loop).")
    ap.add_argument("--demo-delay", type=float, default=6.0, help="Seconds to wait before --demo-help.")
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s  %(name)-9s  %(levelname)-7s  %(message)s")

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
