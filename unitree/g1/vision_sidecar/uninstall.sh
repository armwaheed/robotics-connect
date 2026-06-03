#!/usr/bin/env bash
#
# uninstall.sh — roll back the vision sidecar install to zero.
#
# After this runs the host is byte-for-byte what it was before
# `install.sh` ran, modulo the docker image layer cache (image removal
# is optional via REMOVE_IMAGE=1).

set -euo pipefail

UNIT_NAME="robotics-connect-vision-sidecar.service"
IMAGE="robotics-connect/vision-sidecar:0.1"

say() { printf '\033[1;36m[uninstall]\033[0m %s\n' "$*"; }

say "Stopping + disabling ${UNIT_NAME}"
sudo systemctl stop "${UNIT_NAME}" 2>/dev/null || true
sudo systemctl disable "${UNIT_NAME}" 2>/dev/null || true

say "Removing unit file"
sudo rm -f "/etc/systemd/system/${UNIT_NAME}"
sudo systemctl daemon-reload

say "Removing any lingering container"
docker rm -f robotics-connect-vision-sidecar 2>/dev/null || true

if [[ "${REMOVE_IMAGE:-0}" == "1" ]]; then
    say "Removing image ${IMAGE}"
    docker rmi "${IMAGE}" 2>/dev/null || true
fi

say "Done."
