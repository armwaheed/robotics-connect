#!/bin/bash
#
# This script configures this computer to communicate with the robot's
# private ethernet network by adding a persistent static route to the
# active WiFi connection using NetworkManager's nmcli.
#
# VERSION 4: Uses nmcli for a more robust and direct configuration.
#
# SCOPE: this is host <-> robot NETWORKING only (so the host can reach the
# robot's DDS topics). For bringing up the DGX Spark's Isaac Sim + Isaac Lab
# stack (the GB10/aarch64 gotchas: source build, LD_PRELOAD libgomp, the rsl_rl
# shim, no onnxruntime-GPU, Fabric render), see the `setup-dgx-spark` skill:
#   ../../skills/setup-dgx-spark/SKILL.md  (and its scripts/spark_env.sh)

set -e
set -u

ROBOT_WIFI_IP="192.168.1.119"
ROBOT_SUBNET="192.168.123.0/24"

echo "[*] Starting Robotics Connect network configuration (v4)..."

# --- Step 1: Find the active WiFi connection ---
echo "[1/3] Finding active WiFi connection..."
# Get the NAME of the active connection on a wifi device
CONNECTION_NAME=$(nmcli -t -f NAME,TYPE c show --active | grep -E ':(wifi|802-11-wireless)' | cut -d: -f1)

if [ -z "$CONNECTION_NAME" ]; then
    echo "    -> ERROR: Could not find an active WiFi connection." >&2
    echo "    -> Please ensure you are connected to a WiFi network and try again." >&2
    exit 1
fi
echo "    -> Found active connection: '$CONNECTION_NAME'"

# --- Step 2: Add the static route ---
echo "[2/3] Adding static route to '$CONNECTION_NAME'..."

# Check if the route already exists in the connection profile
EXISTING_ROUTES=$(nmcli -g ipv4.routes c show "$CONNECTION_NAME")
if echo "$EXISTING_ROUTES" | tr ' ' '\n' | grep -q "${ROBOT_SUBNET}"; then
    echo "    -> Route to '$ROBOT_SUBNET' already exists. No changes made."
else
    sudo nmcli connection modify "$CONNECTION_NAME" +ipv4.routes "${ROBOT_SUBNET} ${ROBOT_WIFI_IP}"
    echo "    -> Route added to profile."
fi

# --- Step 3: Re-activate connection to apply changes ---
echo "[3/3] Re-activating connection to apply route..."
# Deactivate and reactivate the connection to make the route take effect immediately.
# The 'sleep 1' gives the 'down' command a moment to complete.
sudo nmcli connection down "$CONNECTION_NAME" && sleep 1 && sudo nmcli connection up "$CONNECTION_NAME"
echo "    -> Connection re-activated."
echo ""
echo "[SUCCESS] Network configuration complete."
echo "You should now be able to ping the robot's ethernet interface:"
echo "ping 192.168.123.164"
