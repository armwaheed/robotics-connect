#!/usr/bin/env python3
"""Human Agent — a person on a Bluetooth headset, exposed as a Device Connect device.

Registers a ``device_type="human_agent"`` device on the Arm Device Connect fabric so a robot
(or any agent) can **consult a human exactly like any other device**: discover it, call
``ask(...)``, and get a grounded decision back. The human hears the question in the headset
earpiece (local TTS) and answers out loud; the answer is transcribed (faster-whisper) on the
compute node and grounded to one of the expected choices.

This is issue armwaheed/robots#3's "Device Connect orchestrates the human interaction": the G1
EDU speaks through its OWN chest speaker (Unitree ``AudioClient`` over DDS — see
``unitree/g1/voice``) and **hears through this Human Agent**, sidestepping the EDU's closed
on-board mic. Register it over NATS with a creds file and it appears live on the Device Connect
dashboard next to the robots, with its callable functions and event stream.

RPCs (callable over Device Connect):
  ask(question, choices="yesno", listen_seconds, retries) -> {heard, transcript, choice, confidence}
  notify(message)                                          -> {spoken}
  presence()                                               -> headset / mic status

Event:
  human_replied(question, choice, transcript)             -> emitted after every grounded answer

Run it (dashboard):
  DEVICE_CONNECT_ALLOW_INSECURE=false \
  python human_agent.py --creds /path/to/.credentials/beta-human-agent-0.creds.json

Try it with no fabric (local headset round-trip only):
  python human_agent.py --self-test
"""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import os
import re
import sys
from typing import Dict, List, Optional, Sequence, Union

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)                       # for bt_headset
sys.path.insert(0, os.path.dirname(_HERE))      # repo root, for dc_sidecar
from bt_headset import RATE, Headset, discover  # noqa: E402
from dc_sidecar import (  # noqa: E402
    HAVE_DC, DeviceDriver, rpc, emit, DeviceIdentity, DeviceStatus, build_runtime, DEFAULT_NATS_URL,
)

log = logging.getLogger("human-agent")


# ══════════════════════════════════════════════════════════════════════════════════════════════
#  Grounding — free speech → one of a small set of expected answers (KnowNo-style)
#  (mirrors unitree/g1/voice/g1_voice.py so the robot's own mic and the headset ground identically)
# ══════════════════════════════════════════════════════════════════════════════════════════════

YES_NO = {
    "yes": ["yes", "yeah", "yep", "yup", "sure", "ok", "okay", "go", "go ahead", "do it",
            "affirmative", "i can", "got it", "done", "ready"],
    "no": ["no", "nope", "not", "don't", "do not", "stop", "wait", "hold on", "negative",
           "can't", "cannot"],
}


def ground(transcript: str, choices: Dict[str, List[str]]):
    """Ground free speech to one of ``choices`` (label → keywords) by keyword overlap.

    Punctuation is stripped first — ASR returns ``"Yes."`` with a period, which must still match the
    ``yes`` keyword. Returns ``(label, confidence)``; ``(None, 0.0)`` if nothing matched (re-ask)."""
    tokens = re.sub(r"[^0-9a-z\s]", " ", transcript.lower()).split()
    t = " " + " ".join(tokens) + " "
    best_label, best_score = None, 0.0
    for label, kws in choices.items():
        hits = sum(1 for kw in kws
                   if (" " + " ".join(kw.lower().split()) + " ") in t or kw.lower() in tokens)
        if hits:
            score = min(1.0, hits / max(1, len(kws)) + 0.001 * hits)
            if score > best_score:
                best_label, best_score = label, score
    return best_label, best_score


def _resolve_choices(choices) -> Optional[Dict[str, List[str]]]:
    if choices is None:
        return None
    if choices == "yesno" or choices == "":
        return dict(YES_NO)
    if isinstance(choices, dict):
        return {k: list(v) for k, v in choices.items()}
    return {label: [label] for label in choices}  # list of labels → each is its own keyword


# ══════════════════════════════════════════════════════════════════════════════════════════════
#  ASR — faster-whisper on the compute node (manual keyboard fallback off-headset)
# ══════════════════════════════════════════════════════════════════════════════════════════════

def make_asr(backend: str = "auto", model: str = "base.en"):
    """Return transcribe(pcm, rate)->str. backend: whisper | manual | auto (whisper→manual)."""
    order = {"auto": ["whisper", "manual"], "whisper": ["whisper"], "manual": ["manual"]}.get(
        backend, [backend])
    for name in order:
        try:
            if name == "whisper":
                from faster_whisper import WhisperModel
                wm = WhisperModel(model, device="auto", compute_type="int8")

                def transcribe(pcm: bytes, rate: int) -> str:
                    import numpy as np
                    if not pcm:
                        return ""
                    # faster-whisper expects 16 kHz float32 mono (our RATE) and has no
                    # sampling_rate kwarg; `rate` is part of the ASRFn interface, not passed here.
                    audio = np.frombuffer(pcm, dtype=np.int16).astype(np.float32) / 32768.0
                    segs, _ = wm.transcribe(audio, language="en")
                    return " ".join(s.text for s in segs).strip()

                return transcribe
            if name == "manual":
                def transcribe(pcm: bytes, rate: int) -> str:
                    try:
                        return input("[human-agent] (manual ASR) type the human's reply > ").strip()
                    except EOFError:
                        return ""
                return transcribe
        except Exception as exc:
            log.warning("ASR backend %s unavailable (%r)", name, exc)
            continue
    raise RuntimeError("no ASR backend")


# ══════════════════════════════════════════════════════════════════════════════════════════════
#  The Device Connect driver  (DeviceDriver/rpc/emit/identity/status come from dc_sidecar)
# ══════════════════════════════════════════════════════════════════════════════════════════════


class HumanAgentDriver(DeviceDriver):
    """A human partner on a Bluetooth headset, as a Device Connect device."""

    device_type = "human_agent"

    def __init__(self, headset: Optional[Headset] = None,
                 display_name: str = "Bluetooth Headset Human Agent",
                 operator_name: str = "Human Operator",
                 earpiece_prompts: bool = True,
                 asr_backend: str = "auto", whisper_model: str = "base.en"):
        if HAVE_DC:
            super().__init__()
        self.headset = headset or Headset()
        self.display_name = display_name
        self.operator_name = operator_name
        # Whether ask() echoes the question into the EARPIECE. In the integrated robot flow the
        # robot speaks the question through its OWN speaker, so the robot agent calls
        # ask(prompt_earpiece=False); standalone (no robot) it defaults True so the human hears it.
        self.earpiece_prompts = earpiece_prompts
        self._asr_backend = asr_backend
        self._whisper_model = whisper_model
        self._asr = None  # lazy: loading whisper takes a moment

    # -- identity / status (what the dashboard shows) ------------------------------------------
    @property
    def identity(self) -> "DeviceIdentity":
        return DeviceIdentity(
            device_type="human_agent",
            manufacturer="robotics-connect",
            model=self.headset.ep.bt_name or "bluetooth-headset",
            description=(f"{self.display_name} — a human partner reachable over a Bluetooth "
                         f"headset on the DGX Spark. A robot asks for help over Device Connect; "
                         f"the human hears it (robot speaker, or earpiece) and answers out loud, "
                         f"transcribed and grounded on-device."),
        )

    @property
    def status(self) -> "DeviceStatus":
        return DeviceStatus(availability="available" if self.headset.present() else "unavailable")

    # -- event ----------------------------------------------------------------------------------
    @emit()
    async def human_replied(self, question: str, choice: Optional[str], transcript: str):
        """Emitted after every grounded answer (shows up in the dashboard event stream)."""

    # -- RPCs -----------------------------------------------------------------------------------
    @rpc()
    async def ask(self, question: str,
                  choices: Union[str, Sequence[str], Dict[str, List[str]]] = "yesno",
                  listen_seconds: float = 7.0, retries: int = 1,
                  prompt_earpiece: Optional[bool] = None) -> dict:
        """Ask the human a question over the headset and return the grounded answer.

        Records the spoken reply, transcribes it, and grounds it to ``choices`` ("yesno" | a list
        of labels | {label: [keywords]} | None for the raw transcript). Re-asks up to ``retries``
        times on an unclear reply. ``prompt_earpiece`` echoes the question into the earpiece
        (default: the agent's ``earpiece_prompts`` setting); the integrated robot flow passes
        False because the robot speaks the question through its own speaker."""
        if prompt_earpiece is None:
            prompt_earpiece = self.earpiece_prompts
        loop = asyncio.get_running_loop()
        res = await loop.run_in_executor(
            None, self._ask_blocking, question, choices, listen_seconds, retries, prompt_earpiece)
        try:
            await self.human_replied(question=question, choice=res.get("choice"),
                                     transcript=res.get("transcript", ""))
        except Exception:
            pass
        return res

    @rpc()
    async def notify(self, message: str) -> dict:
        """Speak a one-way status message into the human's earpiece (no reply expected)."""
        loop = asyncio.get_running_loop()
        spoken = await loop.run_in_executor(None, self.headset.say, message)
        return {"spoken": bool(spoken), "message": message}

    @rpc()
    async def presence(self) -> dict:
        """Report whether the human/headset is reachable, plus the resolved audio routing."""
        info = self.headset.info()
        info["present"] = self.headset.present()
        info["operator"] = self.operator_name
        return info

    # -- blocking worker (audio + ASR are synchronous; run off the event loop) -----------------
    def _ensure_asr(self):
        if self._asr is None:
            self._asr = make_asr(self._asr_backend, self._whisper_model)
        return self._asr

    def _ask_blocking(self, question, choices, listen_seconds, retries, prompt_earpiece=True) -> dict:
        opts = _resolve_choices(choices)
        asr = self._ensure_asr()
        result = {"question": question, "transcript": "", "choice": None,
                  "confidence": 0.0, "heard": False, "options": list((opts or {}).keys())}
        for attempt in range(int(retries) + 1):
            prompt = question if attempt == 0 else f"Sorry, I didn't catch that. {question}"
            if prompt_earpiece:
                self.headset.say(prompt)                   # earpiece (mic gated until done)
            self.headset.play_tone()                       # "your turn" earcon, then listen
            pcm = self.headset.record_utterance(max_seconds=listen_seconds)
            transcript = asr(pcm, RATE) if pcm else ""
            result["transcript"] = transcript
            result["heard"] = bool(transcript)
            if opts is None:
                return result
            label, conf = ground(transcript, opts)
            result["choice"], result["confidence"] = label, conf
            if label is not None:
                return result
        return result


# ══════════════════════════════════════════════════════════════════════════════════════════════
#  Sidecar entry point  (creds + URL resolution + DeviceRuntime live in dc_sidecar.build_runtime)
# ══════════════════════════════════════════════════════════════════════════════════════════════


async def _run_fabric(args) -> None:
    headset = Headset()
    driver = HumanAgentDriver(headset, display_name=args.name, operator_name=args.operator,
                              earpiece_prompts=args.earpiece,
                              asr_backend=args.asr, whisper_model=args.whisper_model)
    rt, creds, urls = build_runtime(driver, args.creds, device_id=args.device_id,
                                    nats_url=args.nats_url)
    log.info("Human Agent '%s' (%s) — %s", creds.get("device_id"), args.name, headset.ep.describe())
    log.info("Registering on the Device Connect fabric %s (tenant=%s)...",
             urls, creds.get("tenant", "default"))
    if args.announce:
        # Greet the human so they know the agent is live and the earpiece works.
        try:
            headset.say(f"Human agent online. {args.operator}, you are connected to Device Connect.")
        except Exception:
            pass
    await rt.run()


def _self_test(args) -> None:
    """No fabric — just exercise the headset ask() round-trip locally."""
    headset = Headset()
    print("Endpoint:", headset.ep.describe())
    driver = HumanAgentDriver(headset, display_name=args.name, operator_name=args.operator,
                              asr_backend=args.asr, whisper_model=args.whisper_model)
    res = driver._ask_blocking(args.question, "yesno", args.listen_seconds, 1, prompt_earpiece=True)
    print(json.dumps(res, indent=2))


def main() -> None:
    ap = argparse.ArgumentParser(description="Human Agent — a person on a Bluetooth headset as a "
                                             "Device Connect device.")
    ap.add_argument("--creds", help="Path to the NATS creds JSON (registers on the dashboard fabric).")
    ap.add_argument("--device-id", default=None, help="Override the device id (default: from creds).")
    ap.add_argument("--nats-url", default=None, help=f"Override NATS url (default: creds / {DEFAULT_NATS_URL}).")
    ap.add_argument("--name", default=os.environ.get("HUMAN_AGENT_NAME", "Bluetooth Headset Human Agent"),
                    help="Agent display name (shown in the device identity on the dashboard).")
    ap.add_argument("--operator", default=os.environ.get("HUMAN_AGENT_OPERATOR", "Human Operator"),
                    help="The person's name (shown in the device identity).")
    ap.add_argument("--earpiece", dest="earpiece", action="store_true", default=True,
                    help="Echo questions into the earpiece (default on for standalone use).")
    ap.add_argument("--no-earpiece", dest="earpiece", action="store_false",
                    help="Don't echo into the earpiece (the robot speaks the question via its speaker).")
    ap.add_argument("--asr", default="auto", help="ASR backend: auto | whisper | manual")
    ap.add_argument("--whisper-model", default="base.en", help="faster-whisper model (e.g. base.en, small.en).")
    ap.add_argument("--announce", action="store_true", help="Greet the human in the earpiece on startup.")
    ap.add_argument("--self-test", action="store_true", help="Run one local ask() round-trip, no fabric.")
    ap.add_argument("--question", default="Can you hold the far corner of the sheet for me?")
    ap.add_argument("--listen-seconds", type=float, default=7.0)
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s  %(name)-14s  %(levelname)-7s  %(message)s")

    if args.self_test or not args.creds:
        if not args.self_test:
            print("No --creds given; running --self-test (local headset round-trip, no dashboard).\n"
                  "Provide --creds <beta-human-agent-*.creds.json> to register on the dashboard.")
        _self_test(args)
        return
    try:
        asyncio.run(_run_fabric(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
