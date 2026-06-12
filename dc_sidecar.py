#!/usr/bin/env python3
"""Shared Device Connect sidecar helpers.

The boilerplate that every Device Connect device sidecar in robotics-connect shares — the SDK
import-shim and the credential/URL resolution — lives HERE, in one place, so the drivers
(`human_agent/human_agent.py`, `unitree/g1/device_connect/g1_agent.py`, and any future robot agent)
don't carry their own divergent copies.

  from dc_sidecar import (HAVE_DC, DeviceDriver, rpc, emit, DeviceIdentity, DeviceStatus,
                          build_runtime, DEFAULT_NATS_URL)
"""

from __future__ import annotations

import json
import os
from typing import Optional, Tuple

# The hosted Arm Device Connect fabric (the dashboard). Creds files may embed an in-cluster
# hostname (nats://nats:4222) that isn't routable off the cluster; resolve_urls() corrects that.
DEFAULT_NATS_URL = "nats://fabric.deviceconnect.dev:4222"


# ── SDK import-shim ───────────────────────────────────────────────────────────────────────────
# device-connect-edge requires Python >=3.11. Importing it lazily behind a shim lets a driver still
# import (and run offline self-tests) on a host without it — e.g. the SDK conda env on a robot.
try:
    from device_connect_edge import DeviceRuntime
    from device_connect_edge.drivers import DeviceDriver, rpc, emit
    from device_connect_edge.types import DeviceIdentity, DeviceStatus

    HAVE_DC = True
except Exception:  # device-connect-edge not installed in this env
    HAVE_DC = False
    DeviceRuntime = None  # type: ignore

    class DeviceDriver:  # type: ignore  minimal stand-in so subclasses import
        pass

    def rpc(*args, **kwargs):  # type: ignore
        def _decorate(fn):
            return fn
        return _decorate

    emit = rpc  # type: ignore

    class _Payload:  # tolerant stand-in for DeviceIdentity / DeviceStatus when the SDK is absent
        def __init__(self, **fields):
            self.__dict__.update(fields)

    DeviceIdentity = DeviceStatus = _Payload  # type: ignore


# ── creds / URL resolution ────────────────────────────────────────────────────────────────────
def load_creds(path: str) -> dict:
    """Load a Device Connect NATS creds JSON ({device_id, tenant, nats:{urls, jwt, nkey_seed}})."""
    with open(os.path.expanduser(path)) as f:
        return json.load(f)


def resolve_urls(creds: dict, override: Optional[str] = None) -> list:
    """The NATS URLs to register on. An explicit `override` wins; otherwise use the creds' urls but
    drop in-cluster hostnames (``nats://nats:4222``) that don't resolve off the cluster, and fall
    back to the public fabric. (This is the one correct copy — both drivers used to differ here.)"""
    if override:
        return [override]
    urls = (creds.get("nats") or {}).get("urls") or []
    routable = [u for u in urls if "://nats:" not in u and "://nats/" not in u]
    return routable or [DEFAULT_NATS_URL]


def build_runtime(driver, creds_path: str, *, device_id: Optional[str] = None,
                  nats_url: Optional[str] = None, tenant: Optional[str] = None
                  ) -> Tuple["DeviceRuntime", dict, list]:
    """Build a `DeviceRuntime` that registers `driver` on the fabric from a creds file.

    Returns ``(runtime, creds, urls)`` WITHOUT running it, so the caller controls the run loop
    (e.g. a one-shot demo that interleaves an RPC with ``runtime.run()``)."""
    if not HAVE_DC:
        raise RuntimeError("device-connect-edge is not installed in this environment "
                           "(needs Python >=3.11 — see the bootstrap-device-connect-env skill).")
    creds = load_creds(creds_path)
    device_id = device_id or creds.get("device_id")
    urls = resolve_urls(creds, nats_url)
    runtime = DeviceRuntime(
        driver=driver, device_id=device_id, messaging_backend="nats", messaging_urls=urls,
        credentials_file=os.path.expanduser(creds_path), tenant=tenant or creds.get("tenant", "default"),
    )
    return runtime, creds, urls
