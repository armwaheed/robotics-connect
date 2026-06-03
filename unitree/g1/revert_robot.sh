#!/bin/bash
#
# This script reverts the network configuration changes made by
# configure_robot.sh. It disables IP forwarding and removes the
# specific firewall rules, restoring the system to its previous state.

set -e
set -u

SYSCTL_CONF="/etc/sysctl.conf"
# Find the most recent backup
SYSCTL_BACKUP=$(ls -t /etc/sysctl.conf.bak-* 2>/dev/null | head -n 1)
IPTABLES_RULES="/etc/iptables/rules.v4"
IPTABLES_BACKUP=$(ls -t /etc/iptables/rules.v4.bak-* 2>/dev/null | head -n 1)


echo "[*] Reverting robot network configuration..."

# --- Step 1: Disable IP Forwarding ---
echo "[1/2] Disabling IP forwarding..."
if [ -f "$SYSCTL_BACKUP" ]; then
    sudo cp "$SYSCTL_BACKUP" "$SYSCTL_CONF"
    sudo sysctl -p "$SYSCTL_CONF" > /dev/null
    echo "    -> Restored $SYSCTL_CONF from backup: $SYSCTL_BACKUP"
    # Optional: remove the backup file after restoring
    # sudo rm "$SYSCTL_BACKUP"
else
    # Fallback if no backup is found: comment out the line
    sudo sed -i '/^net.ipv4.ip_forward=1/s/^/#/' "$SYSCTL_CONF"
    sudo sysctl -p "$SYSCTL_CONF" > /dev/null
    echo "    -> No backup found. Commented out forwarding rule in $SYSCTL_CONF."
fi

# --- Step 2: Remove Firewall Rules ---
echo "[2/2] Removing custom firewall rules..."
if [ -f "$IPTABLES_BACKUP" ]; then
    sudo cp "$IPTABLES_BACKUP" "$IPTABLES_RULES"
    sudo netfilter-persistent reload > /dev/null
    echo "    -> Restored firewall rules from backup: $IPTABLES_BACKUP"
    # Optional: remove the backup file after restoring
    # sudo rm "$IPTABLES_BACKUP"
else
    # Fallback if no backup is found: flush rules and set default policy
    sudo iptables -F FORWARD
    sudo iptables -P FORWARD ACCEPT
    sudo netfilter-persistent save > /dev/null
    echo "    -> No backup found. Flushed FORWARD rules and set policy to ACCEPT."
fi


echo ""
echo "[SUCCESS] Robot configuration has been reverted."

