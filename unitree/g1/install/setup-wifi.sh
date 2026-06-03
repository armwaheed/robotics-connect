#!/usr/bin/env bash
#
# setup-wifi.sh — One-time: enable WiFi persistently on a fresh G1 EDU.
#
# The factory G1 EDU ships with the Realtek rtl8852bu driver PACKAGED
# in /home/unitree/wifi-bt-deb/ but NOT installed. This script installs
# the .deb, copies the firmware, and enables the nvwifibt.service that
# brings WiFi up on every boot.
#
# Run ONCE per robot over ethernet. After this, reboot the robot and
# WiFi comes up automatically.
#
# Fully idempotent: re-running on an already-configured robot is a
# no-op.
#
# Driver package lookup (first match wins):
#   1. /home/unitree/wifi-bt-deb/                         (factory)
#   2. ${HERE}/wifi-bt-deb/                               (repo bundle, optional)
#   3. ${OFFLINE_DIR}/wifi-bt-deb/                        (offline bundle)
#   4. $WIFI_BT_DEB_DIR (env override)                    (manual)
#
# Reference: https://docs.westonrobot.com/tutorial/unitree/g1_internet_guide/

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${HERE}/.." && pwd)"
OFFLINE_DIR="${OFFLINE_DIR:-${REPO_ROOT}/../offline}"

say() { printf '\033[1;36m[wifi-setup]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[wifi-setup]\033[0m %s\n' "$*" >&2; }
die() { printf '\033[1;31m[wifi-setup]\033[0m %s\n' "$*" >&2; exit 1; }

if [[ "${EUID}" -eq 0 ]]; then
    SUDO=""
else
    SUDO="sudo -E "
fi

# ── Idempotency check 1: fully configured already? ──────────────────────

if systemctl is-enabled --quiet nvwifibt.service 2>/dev/null \
   && ip -br link show wlan0 2>/dev/null | grep -q UP; then
    say "WiFi is already persistent (nvwifibt.service enabled, wlan0 UP). No-op."
    exit 0
fi

# ── Idempotency check 2: driver installed but service just not running? ─

if dpkg -l rtl8852bu-dkms 2>/dev/null | awk 'NR>5 && $1=="ii"{exit 0} END{exit 1}'; then
    say "rtl8852bu-dkms already installed — just need to (re-)enable the service"
    skip_dpkg=1
else
    skip_dpkg=0
fi

# ── Locate the driver package if we need it ─────────────────────────────

deb_dir=""
if [[ "${skip_dpkg}" == "0" ]]; then
    candidates=(
        "${WIFI_BT_DEB_DIR:-}"                 # operator-provided override
        "/home/unitree/wifi-bt-deb"            # factory
        "${HERE}/wifi-bt-deb"                  # repo bundle (gitignored)
        "${OFFLINE_DIR}/wifi-bt-deb"           # offline bundle
    )
    for c in "${candidates[@]}"; do
        [[ -z "$c" ]] && continue
        if [[ -d "$c" ]] && find "$c" -maxdepth 1 -name 'rtl8852bu-dkms_*.deb' | grep -q .; then
            deb_dir="$c"
            break
        fi
    done
    [[ -n "${deb_dir}" ]] || die \
        "No rtl8852bu-dkms driver package found in any known location:
    - /home/unitree/wifi-bt-deb/    (factory — usually present on stock G1 EDU)
    - ${HERE}/wifi-bt-deb/
    - ${OFFLINE_DIR}/wifi-bt-deb/
    - \$WIFI_BT_DEB_DIR (env var override, unset)
  Obtain the package from your G1 vendor or a known-good G1 EDU and
  drop it at one of those paths, or point WIFI_BT_DEB_DIR at its directory."

    say "Using driver package from ${deb_dir}"
fi

# ── Install driver + firmware + enable service ──────────────────────────

if [[ "${skip_dpkg}" == "0" ]]; then
    deb="$(find "${deb_dir}" -maxdepth 1 -name 'rtl8852bu-dkms_*.deb' | head -1)"
    say "Installing ${deb}"
    ${SUDO}dpkg -i "${deb}"

    if [[ -x "${deb_dir}/copy_firmware.sh" ]]; then
        say "Running ${deb_dir}/copy_firmware.sh"
        ${SUDO}bash "${deb_dir}/copy_firmware.sh"
    fi
fi

if systemctl is-enabled --quiet nvwifibt.service 2>/dev/null; then
    say "nvwifibt.service already enabled"
else
    say "Enabling nvwifibt.service"
    ${SUDO}systemctl enable nvwifibt.service
fi

# restart is idempotent whether the service was up or down
${SUDO}systemctl restart nvwifibt.service

# ── Verify wlan0 came up ────────────────────────────────────────────────

for _ in 1 2 3 4 5; do
    if ip -br link show wlan0 2>/dev/null | grep -q UP; then
        say "wlan0 is UP."
        say "Connect to your SSID with:  nmcli device wifi connect <ssid> password <pw>"
        exit 0
    fi
    sleep 1
done

die "wlan0 did not come UP within 5s. Check 'journalctl -u nvwifibt -n 50' and 'dmesg | tail'."
