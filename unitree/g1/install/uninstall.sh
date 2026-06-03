#!/usr/bin/env bash
#
# uninstall.sh — Remove Robotics Connect from the robot. Factory state returns.
#
# After this runs, the robot is byte-for-byte a factory G1 EDU again —
# modulo the conda package cache (which is preserved for performance).
#
# Does NOT touch any factory artifacts listed in install.sh.

set -euo pipefail

INSTALL_DIR="/home/unitree/robotics-connect"
CONDA_ENV="robotics-connect"
PROFILE_HOOK="/etc/profile.d/robotics-connect.sh"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

say() { printf '\033[1;36m[uninstall]\033[0m %s\n' "$*"; }

if [[ "${EUID}" -eq 0 ]]; then
    SUDO=""
else
    SUDO="sudo -E "
fi

# ── Stop and remove any robotics-connect-* systemd units ────────────────
# We enumerate by prefix so future services are cleaned up automatically.

say "Stopping robotics-connect-* systemd units"
mapfile -t units < <(systemctl list-unit-files 'robotics-connect-*' --no-legend 2>/dev/null | awk '{print $1}')
for u in "${units[@]}"; do
    say "  - ${u}"
    ${SUDO}systemctl stop "$u" 2>/dev/null || true
    ${SUDO}systemctl disable "$u" 2>/dev/null || true
    ${SUDO}rm -f "/etc/systemd/system/$u"
done
${SUDO}systemctl daemon-reload

# ── Vision sidecar: delegate to its own uninstaller for the docker bits ─

if [[ -x "${INSTALL_DIR}/vision_sidecar/uninstall.sh" ]]; then
    say "Running vision sidecar uninstall"
    (cd "${INSTALL_DIR}/vision_sidecar" && ./uninstall.sh) || true
fi

# ── Remove conda env ────────────────────────────────────────────────────

# shellcheck source=/dev/null
source /home/unitree/miniconda3/etc/profile.d/conda.sh
if conda env list | awk '{print $1}' | grep -qx "${CONDA_ENV}"; then
    say "Removing conda env ${CONDA_ENV}"
    conda env remove --name "${CONDA_ENV}" --yes
fi

# ── Remove code tree ────────────────────────────────────────────────────

if [[ -d "${INSTALL_DIR}" ]]; then
    say "Removing ${INSTALL_DIR}"
    rm -rf "${INSTALL_DIR}"
fi

# ── Remove profile hook ─────────────────────────────────────────────────

if [[ -f "${PROFILE_HOOK}" ]]; then
    say "Removing ${PROFILE_HOOK}"
    ${SUDO}rm -f "${PROFILE_HOOK}"
fi

say "Done. Robot is back to factory state."
