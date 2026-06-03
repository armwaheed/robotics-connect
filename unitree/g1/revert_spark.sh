#!/bin/bash
#
# This script reverts the network configuration changes by removing the
# static route from the active WiFi connection using NetworkManager's nmcli.
#
# VERSION 4: Uses nmcli for a more robust and direct configuration.

set -e
set -u

ROBOT_WIFI_IP="192.168.1.119"
ROBOT_SUBNET="192.168.123.0/24"

echo "[*] Reverting Robotics Connect network configuration (v4)..."

# --- Step 1: Find the active WiFi connection ---
echo "[1/3] Finding active WiFi connection..."
CONNECTION_NAME=$(nmcli -t -f NAME,TYPE c show --active | grep -E ':(wifi|802-11-wireless)' | cut -d: -f1)

if [ -z "$CONNECTION_NAME" ]; then
    echo "    -> WARNING: Could not find an active WiFi connection. No changes to make."
    exit 0
fi
echo "    -> Found active connection: '$CONNECTION_NAME'"

# --- Step 2: Remove the static route ---
echo "[2/3] Removing static route from '$CONNECTION_NAME'..."

EXISTING_ROUTES=$(nmcli -g ipv4.routes c show "$CONNECTION_NAME")
if echo "$EXISTING_ROUTES" | tr ' ' '\n' | grep -q "${ROBOT_SUBNET}"; then
    sudo nmcli connection modify "$CONNECTION_NAME" -ipv4.routes "${ROBOT_SUBNET} ${ROBOT_WIFI_IP}"
    echo "    -> Route removed from profile."
else
    echo "    -> Route to '$ROBOT_SUBNET' does not exist. No changes made."
fi

# --- Step 3: Re-activate connection to apply changes ---
echo "[3/3] Re-activating connection to apply changes..."
sudo nmcli connection down "$CONNECTION_NAME" && sleep 1 && sudo nmcli connection up "$CONNECTION_NAME"
echo "    -> Connection re-activated."
echo ""
echo "[SUCCESS] Network configuration has been reverted."
