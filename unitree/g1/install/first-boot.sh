#!/usr/bin/env bash
#
# first-boot.sh — One-command Robotics Connect bootstrap for a fresh Unitree G1 EDU.
#
# Goes from "just got the robot" to "ready to run the stack":
#   1. enable the WiFi driver persistently,
#   2. associate wlan0 with your home/lab SSID,
#   3. run the full install.sh.
#
# FULLY IDEMPOTENT. If any step is already done, that step is a no-op.
# Running first-boot.sh against a robot that is already fully set up is
# safe — it prints a green summary and exits.
#
# Usage:
#   # Interactive (prompts for SSID/password if not supplied):
#   ./first-boot.sh
#
#   # Non-interactive / scripted:
#   SSID="MyNet" WIFI_PASS="secret" ./first-boot.sh
#
#   # Skip the WiFi-join step (useful for ethernet-only operators):
#   SKIP_WIFI_JOIN=1 ./first-boot.sh
#
#   # Also install the GPU vision sidecar (DINOv2, ~12 GB; only needed for
#   # GPU embedding workloads):
#   WITH_SIDECAR=1 ./first-boot.sh
#
# Run ON THE ROBOT after ethernet-copying the bundle over.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

say()   { printf '\033[1;36m[first-boot]\033[0m %s\n' "$*"; }
win()   { printf '\033[1;32m[first-boot]\033[0m %s\n' "$*"; }
warn()  { printf '\033[1;33m[first-boot]\033[0m %s\n' "$*" >&2; }
die()   { printf '\033[1;31m[first-boot]\033[0m %s\n' "$*" >&2; exit 1; }

# ── Step 1: WiFi driver ─────────────────────────────────────────────────

say "Step 1/3 — WiFi driver persistence"
bash "${HERE}/setup-wifi.sh"

# ── Step 2: Associate wlan0 with an SSID ────────────────────────────────

if [[ "${SKIP_WIFI_JOIN:-0}" == "1" ]]; then
    say "Step 2/3 — SKIP_WIFI_JOIN=1, leaving wlan0 unassociated"
else
    say "Step 2/3 — WiFi association"

    # Is wlan0 already associated with a live network?
    if nmcli -t -f DEVICE,STATE device 2>/dev/null | grep -qx 'wlan0:connected'; then
        current="$(nmcli -t -f ACTIVE,SSID device wifi 2>/dev/null | awk -F: '/^yes/{print $2; exit}')"
        say "wlan0 already connected to '${current:-<unknown>}' — skipping join"
    else
        SSID="${SSID:-}"
        WIFI_PASS="${WIFI_PASS:-}"

        if [[ -z "${SSID}" ]]; then
            if [[ ! -t 0 ]]; then
                die "No SSID set and not running interactively.  Set SSID=... WIFI_PASS=... or run with a TTY."
            fi
            read -rp "WiFi SSID: " SSID
            [[ -n "${SSID}" ]] || die "SSID cannot be empty"
            read -rsp "WiFi password (empty for open network): " WIFI_PASS
            echo
        fi

        say "Associating wlan0 with '${SSID}'"
        if [[ -n "${WIFI_PASS}" ]]; then
            sudo nmcli device wifi connect "${SSID}" password "${WIFI_PASS}" ifname wlan0
        else
            sudo nmcli device wifi connect "${SSID}" ifname wlan0
        fi
    fi
fi

# ── Step 3: Install the stack ───────────────────────────────────────────

say "Step 3/3 — Installing Robotics Connect stack"
bash "${HERE}/install.sh"

# ── Summary ─────────────────────────────────────────────────────────────

echo
win "Robotics Connect first-boot complete."
eth_ip="$(ip -4 addr show eth0  2>/dev/null | awk '/inet /{print $2}' | head -1 || true)"
wlan_ip="$(ip -4 addr show wlan0 2>/dev/null | awk '/inet /{print $2}' | head -1 || true)"
[[ -n "${eth_ip}"  ]] && echo "  eth0  : ${eth_ip}"
[[ -n "${wlan_ip}" ]] && echo "  wlan0 : ${wlan_ip}"
echo
echo "  Next time, SSH over WiFi:  ssh unitree@${wlan_ip%/*}"
echo "  Run the FK selftest:       conda activate robotics-connect && \\"
echo "                             python /home/unitree/robotics-connect/arm_fk/arm_fk.py"
