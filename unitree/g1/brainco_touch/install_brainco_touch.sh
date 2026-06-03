#!/usr/bin/env bash
#
# install_brainco_touch.sh — deploy the direct-Modbus Brainco touch bridge
# to a G1 EDU robot, verify it runs, and confirm touch sensors are live.
#
# Usage:
#     ./install_brainco_touch.sh [user@host]
#
# Env:
#     BRAINCO_SSH_PASS   (default: 123)      SSH password for `sshpass`
#     BRAINCO_REMOTE_DIR (default: /home/unitree/brainco_touch)
#
# Prereqs on the robot:
#   * `g1brainco` conda env with `pyserial`
#   * /dev/ttyUSB1 and /dev/ttyUSB2 (left + right hand dongles)
#   * Both hands powered and in the expected slave IDs (0x7e / 0x7f)
#
# Prereqs on your laptop:
#   * sshpass, ssh, scp in PATH
#
# See README.md in this directory for the full story and register layout.

set -euo pipefail

HOST="${1:-unitree@192.168.123.164}"
PASS="${BRAINCO_SSH_PASS:-123}"
REMOTE_DIR="${BRAINCO_REMOTE_DIR:-/home/unitree/brainco_touch}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE_SRC="$SCRIPT_DIR/brainco_bridge.py"

if [[ ! -f "$BRIDGE_SRC" ]]; then
    echo "ERROR: $BRIDGE_SRC not found — run this script from the brainco_touch directory." >&2
    exit 1
fi

SSH_OPTS=(-o StrictHostKeyChecking=no -o ConnectTimeout=10)
SSH="sshpass -p $PASS ssh ${SSH_OPTS[*]} $HOST"
SCP="sshpass -p $PASS scp ${SSH_OPTS[*]}"

echo "==> Deploying brainco_touch bridge to $HOST:$REMOTE_DIR"

# 1. Create remote directory
$SSH "mkdir -p $REMOTE_DIR"

# 2. Copy the bridge, README, and smoke-test script
$SCP "$BRIDGE_SRC" "$HOST:$REMOTE_DIR/brainco_bridge.py"
$SCP "$SCRIPT_DIR/README.md" "$HOST:$REMOTE_DIR/README.md"
$SCP "$SCRIPT_DIR/smoke_test.py" "$HOST:$REMOTE_DIR/smoke_test.py"

# 3. Verify the deployed bridge has the touch-register code
# (grep sentinel: verify the bridge deployed with its touch-register code)
SENTINEL_COUNT=$($SSH "grep -c REG_TOUCH_ENABLE_ADDR $REMOTE_DIR/brainco_bridge.py" || echo 0)
if [[ "$SENTINEL_COUNT" -lt 2 ]]; then
    echo "ERROR: sentinel grep failed (found $SENTINEL_COUNT, expected >=2)" >&2
    exit 1
fi
echo "    ok: touch register code deployed (grep count = $SENTINEL_COUNT)"

# 4. Check pyserial is available in g1brainco
echo "==> Checking pyserial in g1brainco env..."
$SSH '
    source ~/miniconda3/etc/profile.d/conda.sh
    conda activate g1brainco
    python -c "import serial; print(f\"    ok: pyserial {serial.VERSION}\")"
' || {
    echo "ERROR: pyserial not available in g1brainco env.  Install it with:"
    echo "    conda activate g1brainco && pip install pyserial"
    exit 1
}

# 5. Smoke test: stop any existing bridge, start the new one, poll once.
# Can't use `pkill -f brainco_bridge.py` because `-f` matches the pkill's own
# argv (which contains that literal) and kills the SSH session — bash's
# `[p]ython` char-class trick avoids the self-match.
echo "==> Stopping any existing brainco_bridge..."
$SSH "ps axo pid,cmd | awk '/[p]ython.*brainco_bridge\\.py/ {print \$1}' | xargs -r kill 2>/dev/null; true"

echo "==> Starting brainco_bridge in background..."
# Run the bridge in a subshell with all three std fds redirected OUTSIDE the
# subshell, then background+disown.  Without the outer redirection SSH waits
# for the backgrounded process to release the inherited stdout/stderr
# channels and never returns.
$SSH "(source ~/miniconda3/etc/profile.d/conda.sh && conda activate g1brainco && cd $REMOTE_DIR && exec python brainco_bridge.py) </dev/null >/tmp/brainco_bridge.log 2>&1 & disown -a; exit 0"

# Give the bridge a moment to open serial ports and bind the TCP port
sleep 3

echo "==> Verifying bridge log..."
$SSH "cat /tmp/brainco_bridge.log" || {
    echo "ERROR: bridge log missing — bridge likely failed to start" >&2
    exit 1
}

echo "==> Running smoke test..."
$SSH "source ~/miniconda3/etc/profile.d/conda.sh && conda activate g1brainco && python $REMOTE_DIR/smoke_test.py" || {
    echo "ERROR: smoke test failed — check /tmp/brainco_bridge.log on the robot" >&2
    exit 1
}

cat <<EOF

==> Install complete.

Next steps on the robot:
  1. The bridge is currently running in the background on tcp://127.0.0.1:9877.
     Log: /tmp/brainco_bridge.log on the robot.
  2. To stop it (via SSH — avoid 'pkill -f' which self-matches the SSH argv):
        ssh $HOST "ps axo pid,cmd | awk '/[p]ython.*brainco_bridge.py/ {print \\\$1}' | xargs -r kill"
  3. To (re)start it manually in an interactive shell:
        ssh $HOST
        source ~/miniconda3/etc/profile.d/conda.sh
        conda activate g1brainco
        python $REMOTE_DIR/brainco_bridge.py
  4. Protocol: newline-delimited JSON on tcp://127.0.0.1:9877.  See
     $REMOTE_DIR/README.md for field definitions and register layout.

EOF
