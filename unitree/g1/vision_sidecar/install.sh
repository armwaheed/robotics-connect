#!/usr/bin/env bash
#
# install.sh — stand up the Robotics Connect vision sidecar.
#
# Idempotent: safe to re-run. On the first run this:
#   1. checks that docker + nvidia-container-toolkit are present,
#   2. builds robotics-connect/vision-sidecar:0.1 from the local Dockerfile
#      (or loads it from a tarball at /tmp/robotics-connect-vision-sidecar.tar
#       if one is provided — useful for offline customer installs),
#   3. installs /etc/systemd/system/robotics-connect-vision-sidecar.service,
#   4. starts the service and pings it.
#
# On subsequent runs it just re-applies the systemd unit and restarts
# the service. The image is only rebuilt if $REBUILD=1 is set.
#
# This script ONLY touches:
#   - docker images local cache
#   - /etc/systemd/system/robotics-connect-vision-sidecar.service
#
# It does NOT touch:
#   - the unitree_deploy conda env,
#   - /etc/docker/daemon.json,
#   - /usr/local/cuda-*,
#   - the host python install.
#
# Rollback is `./uninstall.sh`.

set -euo pipefail

IMAGE_REPO="robotics-connect/vision-sidecar"
IMAGE_TAG="0.1"
IMAGE="${IMAGE_REPO}:${IMAGE_TAG}"
UNIT_NAME="robotics-connect-vision-sidecar.service"
OFFLINE_TAR="${OFFLINE_TAR:-/tmp/robotics-connect-vision-sidecar.tar}"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Root-requiring steps use `$SUDO` so the script can be driven both ways:
#   - as an unprivileged user: `./install.sh`          (sudo will prompt)
#   - under sudo already:      `sudo bash install.sh`  (SUDO="" skips the wrap)
#
# Default: empty if already root, `sudo -E` otherwise.
if [[ "${EUID}" -eq 0 ]]; then
    SUDO=""
else
    SUDO="sudo -E "
fi

# docker: prefer group-membership access; fall back to sudo when needed
# (e.g. right after usermod -aG docker, before the user re-logs in).
if docker info >/dev/null 2>&1; then
    DOCKER="docker"
else
    DOCKER="${SUDO}docker"
fi

say() { printf '\033[1;36m[install]\033[0m %s\n' "$*"; }
die() { printf '\033[1;31m[install]\033[0m %s\n' "$*" >&2; exit 1; }

require_cmd() {
    command -v "$1" >/dev/null 2>&1 || die "$1 not found on PATH"
}

say "Checking prerequisites"
require_cmd docker
require_cmd systemctl
# nvidia-container-cli is the canonical binary the docker nvidia runtime
# shells out to. If it's missing the container will fail with a cryptic
# error at start time; catch it here instead.
command -v nvidia-container-cli >/dev/null 2>&1 \
    || die "nvidia-container-cli not found — install nvidia-container-toolkit first"

# Runtime registration check — belt-and-braces against a busted daemon.json.
if ! ${DOCKER} info 2>/dev/null | grep -q '^ Runtimes:.*nvidia'; then
    die "docker daemon does not register the 'nvidia' runtime. " \
        "Check /etc/docker/daemon.json and restart docker."
fi

# ── Image: build, load-from-tar, or accept already present ──────────────────

if ${DOCKER} image inspect "${IMAGE}" >/dev/null 2>&1 && [[ "${REBUILD:-0}" != "1" ]]; then
    say "Image ${IMAGE} already present — skipping build (set REBUILD=1 to force)"
elif [[ -f "${OFFLINE_TAR}" ]]; then
    say "Loading image from ${OFFLINE_TAR}"
    ${DOCKER} load -i "${OFFLINE_TAR}"
else
    say "Building image ${IMAGE} from ${HERE}"
    ${DOCKER} build -t "${IMAGE}" "${HERE}"
fi

# ── systemd unit ────────────────────────────────────────────────────────────

say "Installing systemd unit to /etc/systemd/system/${UNIT_NAME}"
${SUDO}install -m 644 "${HERE}/${UNIT_NAME}" "/etc/systemd/system/${UNIT_NAME}"
${SUDO}systemctl daemon-reload
${SUDO}systemctl enable "${UNIT_NAME}"
${SUDO}systemctl restart "${UNIT_NAME}"

# ── Health check via the ping command ───────────────────────────────────────

say "Waiting for the sidecar to answer ping on 127.0.0.1:9878"
python3 - <<'PY' || die "sidecar did not come up cleanly — check 'journalctl -u robotics-connect-vision-sidecar -n 50'"
import json, socket, struct, sys, time
deadline = time.monotonic() + 180.0  # DINOv2 load + first-encode warmup
last_err = None
while time.monotonic() < deadline:
    try:
        s = socket.create_connection(("127.0.0.1", 9878), timeout=2.0)
        hdr = json.dumps({"cmd": "ping"}).encode("utf-8")
        s.sendall(struct.pack("<I", len(hdr)) + hdr)
        (n,) = struct.unpack("<I", s.recv(4))
        buf = b""
        while len(buf) < n:
            buf += s.recv(n - len(buf))
        info = json.loads(buf.decode("utf-8"))
        print(f"  ping ok: {info}")
        s.close()
        sys.exit(0 if info.get("ok") else 1)
    except Exception as e:
        last_err = e
        time.sleep(3.0)
sys.stderr.write(f"timed out waiting for ping, last error: {last_err!r}\n")
sys.exit(1)
PY

say "Done. Sidecar is up and healthy."
