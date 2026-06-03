#!/bin/bash
#
# This script configures the G1 robot to forward network traffic from the
# DGX Spark (192.168.1.110) to the robot's private ethernet network.
# It enables IP forwarding and sets up a firewall to only allow traffic
# from the Spark.

set -e
set -u

SPARK_IP="192.168.1.110"
ROBOT_WLAN_IF="wlan0"
ROBOT_ETH_IF="eth0"
ROBOT_ETH_IP="192.168.123.164"
SYSCTL_CONF="/etc/sysctl.conf"
SYSCTL_BACKUP="/etc/sysctl.conf.bak-$(date +%F)"
IPTABLES_RULES="/etc/iptables/rules.v4"
IPTABLES_BACKUP="/etc/iptables/rules.v4.bak-$(date +%F)"

echo "[*] Starting robot network configuration..."

# --- Step 1: Enable IP Forwarding ---
echo "[1/3] Enabling persistent IP forwarding..."
if [ ! -f "$SYSCTL_BACKUP" ]; then
    sudo cp "$SYSCTL_CONF" "$SYSCTL_BACKUP"
    echo "    -> Created backup: $SYSCTL_BACKUP"
fi

if ! grep -q "net.ipv4.ip_forward=1" "$SYSCTL_CONF"; then
    echo "net.ipv4.ip_forward=1" | sudo tee -a "$SYSCTL_CONF" > /dev/null
    echo "    -> Added 'net.ipv4.ip_forward=1' to $SYSCTL_CONF"
else
    echo "    -> IP forwarding is already enabled in $SYSCTL_CONF."
fi

# Apply the setting
sudo sysctl -p "$SYSCTL_CONF" > /dev/null
echo "    -> Applied sysctl changes."

# --- Step 2: Install and Configure Firewall ---
echo "[2/3] Installing and configuring firewall (iptables)..."
if ! dpkg -s iptables-persistent >/dev/null 2>&1; then
    echo "    -> Installing iptables-persistent..."
    sudo apt-get update
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y iptables-persistent
fi

echo "    -> Backing up current firewall rules to $IPTABLES_BACKUP (if they exist)..."
sudo touch $IPTABLES_RULES # Ensure file exists before copy
sudo cp "$IPTABLES_RULES" "$IPTABLES_BACKUP"

echo "    -> Setting new firewall rules..."
# Flush existing FORWARD rules
sudo iptables -F FORWARD

# Allow forwarding for ESTABLISHED and RELATED connections
sudo iptables -A FORWARD -m state --state ESTABLISHED,RELATED -j ACCEPT

# Allow NEW connections from Spark's WiFi to Robot's Ethernet
sudo iptables -A FORWARD -i "$ROBOT_WLAN_IF" -o "$ROBOT_ETH_IF" -s "$SPARK_IP" -d "$ROBOT_ETH_IP" -j ACCEPT

# Drop all other forwarding traffic
sudo iptables -P FORWARD DROP
echo "    -> Policy set: Allow Spark -> Robot, drop all other forwards."

# --- Step 3: Save Firewall Rules ---
echo "[3/3] Saving firewall rules to make them persistent..."
sudo netfilter-persistent save > /dev/null
echo "    -> Rules saved."

echo ""
echo "[SUCCESS] Robot configuration complete."
echo "You can now proceed with configuring your DGX Spark."

