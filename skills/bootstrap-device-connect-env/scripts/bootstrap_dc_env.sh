#!/usr/bin/env bash
# Bootstrap a clean Python>=3.11 environment for Device Connect (device-connect-edge needs >=3.11).
#
# Conda-or-venv agnostic: uses conda when available (creates an env), else a venv. Idempotent —
# re-running just verifies/updates. This does NOT touch the robot's vendor SDK env; in BRIDGED
# mode (the usual case for a vendor-pinned SDK) the Device Connect sidecar runs HERE and delegates
# hardware calls to the SDK env via subprocess (see references/two-env-bridge.md).
#
# Usage:
#   bash bootstrap_dc_env.sh                 # env name "device-connect", + agent-tools
#   DC_ENV_NAME=dc DC_WITH_AGENT_TOOLS=0 bash bootstrap_dc_env.sh
set -euo pipefail

ENV_NAME="${DC_ENV_NAME:-device-connect}"
PY_VERSION="${DC_PY_VERSION:-3.11}"
WITH_AGENT_TOOLS="${DC_WITH_AGENT_TOOLS:-1}"
EXTRA_PKGS="${DC_EXTRA_PKGS:-}"   # e.g. "faster-whisper piper-tts" for an audio human-agent host

PKGS="device-connect-edge"
[ "$WITH_AGENT_TOOLS" = "1" ] && PKGS="$PKGS device-connect-agent-tools"
[ -n "$EXTRA_PKGS" ] && PKGS="$PKGS $EXTRA_PKGS"

echo ">> Bootstrapping a Python>=${PY_VERSION} env '${ENV_NAME}' for: ${PKGS}"

PYBIN=""
if command -v conda >/dev/null 2>&1; then
    echo ">> conda found — using a conda env."
    # shellcheck disable=SC1091
    source "$(conda info --base)/etc/profile.d/conda.sh"
    if ! conda env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
        conda create -y -n "$ENV_NAME" "python=${PY_VERSION}"
    else
        echo ">> conda env '${ENV_NAME}' already exists — reusing."
    fi
    conda activate "$ENV_NAME"
    PYBIN="$(command -v python)"
else
    echo ">> no conda — using a venv (requires a system python>=${PY_VERSION})."
    SYS_PY=""
    for c in "python${PY_VERSION}" python3.13 python3.12 python3.11 python3; do
        if command -v "$c" >/dev/null 2>&1; then
            v="$("$c" -c 'import sys;print("%d.%d"%sys.version_info[:2])')"
            if [ "$(printf '%s\n%s\n' "$PY_VERSION" "$v" | sort -V | head -1)" = "$PY_VERSION" ]; then
                SYS_PY="$c"; break
            fi
        fi
    done
    if [ -z "$SYS_PY" ]; then
        echo "!! No system python>=${PY_VERSION} found. Install one (e.g. 'sudo apt install python3.11-venv') or install Miniforge, then re-run." >&2
        exit 1
    fi
    VENV_DIR="${HOME}/.venvs/${ENV_NAME}"
    [ -d "$VENV_DIR" ] || "$SYS_PY" -m venv "$VENV_DIR"
    PYBIN="$VENV_DIR/bin/python"
fi

echo ">> Using interpreter: $PYBIN ($("$PYBIN" --version 2>&1))"
"$PYBIN" -m pip install -q --upgrade pip
"$PYBIN" -m pip install -q $PKGS

echo ">> Verifying..."
"$PYBIN" - <<'PY'
import importlib.metadata as m
import device_connect_edge  # noqa: F401
print("   device-connect-edge", m.version("device-connect-edge"), "OK")
try:
    import device_connect_agent_tools  # noqa: F401
    print("   device-connect-agent-tools", m.version("device-connect-agent-tools"), "OK")
except Exception:
    print("   device-connect-agent-tools not installed (ok)")
PY
echo ">> Done. Device Connect interpreter: $PYBIN"
