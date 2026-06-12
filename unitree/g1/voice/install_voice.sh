#!/usr/bin/env bash
#
# install_voice.sh — deploy the G1 voice module to a G1 EDU and verify the speaker is live.
#
# The module FILES ship with the normal robotics-connect installer (install/install.sh deploys the
# whole unitree/g1 tree). This standalone script is for the two extra steps:
#   1. install an optional LOCAL ASR backend (faster-whisper) into the robotics-connect conda env
#      so the robot can transcribe the human's reply on-device (the speaker needs no extra deps —
#      unitree_sdk2py is already on the robot);
#   2. run the voice diagnostic (speak a test phrase + list mic sources).
#
# Usage:
#     ./install_voice.sh [user@host]
#
# Env:
#     VOICE_SSH_PASS    (default: 123)                        SSH password for sshpass
#     VOICE_REMOTE_DIR  (default: /home/unitree/robotics-connect/unitree/g1/voice)
#     VOICE_ENV         (default: robotics-connect)           conda env to install ASR into
#     WITH_ASR          (default: 1)                          1 = pip install faster-whisper
#     VOICE_IFACE       (default: eth0)                        DDS iface on the robot
#
# Prereqs on your laptop: sshpass, ssh, scp in PATH.

set -euo pipefail

HOST="${1:-unitree@192.168.1.119}"          # the WiFi SSH path; override for the 192.168.123.x subnet
PASS="${VOICE_SSH_PASS:-123}"
REMOTE_DIR="${VOICE_REMOTE_DIR:-/home/unitree/robotics-connect/unitree/g1/voice}"
ENV_NAME="${VOICE_ENV:-robotics-connect}"
WITH_ASR="${WITH_ASR:-1}"
IFACE="${VOICE_IFACE:-eth0}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f "$SCRIPT_DIR/g1_voice.py" ]]; then
    echo "ERROR: g1_voice.py not found — run this from the voice/ directory." >&2
    exit 1
fi

SSH_OPTS=(-o StrictHostKeyChecking=no -o ConnectTimeout=10)
SSH="sshpass -p $PASS ssh ${SSH_OPTS[*]} $HOST"
SCP="sshpass -p $PASS scp ${SSH_OPTS[*]}"

echo "[voice] deploying module to $HOST:$REMOTE_DIR"
$SSH "mkdir -p '$REMOTE_DIR'"
$SCP "$SCRIPT_DIR"/g1_voice.py "$SCRIPT_DIR"/_diag_voice.py "$HOST:$REMOTE_DIR/"

if [[ "$WITH_ASR" == "1" ]]; then
    echo "[voice] installing faster-whisper into conda env '$ENV_NAME' (local on-device ASR)"
    # faster-whisper pulls ctranslate2; on the Orin it uses CUDA if available, else CPU int8.
    $SSH "source ~/miniconda3/etc/profile.d/conda.sh 2>/dev/null || source ~/anaconda3/etc/profile.d/conda.sh; \
          conda activate '$ENV_NAME' && pip install -q faster-whisper || \
          echo '[voice] WARN: faster-whisper install failed — listener falls back to manual ASR'"
fi

echo "[voice] running the speaker diagnostic on the robot"
$SSH "source ~/miniconda3/etc/profile.d/conda.sh 2>/dev/null || source ~/anaconda3/etc/profile.d/conda.sh; \
      conda activate '$ENV_NAME'; cd '$REMOTE_DIR' && python _diag_voice.py --iface '$IFACE'" || {
    echo "[voice] diagnostic returned non-zero — check the speaker / 'voice' DDS service (is the built-in VUI holding it?)." >&2
    exit 1
}

echo "[voice] done. The G1 spoke a test phrase; mic sources listed above."
echo "[voice] full ask() round-trip:  python _diag_voice.py --iface $IFACE --ask"
