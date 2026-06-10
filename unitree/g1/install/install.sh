#!/usr/bin/env bash
#
# install.sh — Deploy the Robotics Connect stack onto a fresh Unitree G1 EDU.
#
# Target:  Unitree G1 EDU Type F (Jetson Orin, L4T Ubuntu 20.04).
# Intent:  plug-and-play.
#
# The factory G1 ships with a rich but OFF-by-default stack:
#   WiFi driver not loaded, Robotics Connect code absent, vision sidecar absent.
# This script takes that factory image and installs Robotics Connect on top,
# WITHOUT touching anything Unitree shipped. Rollback is ./uninstall.sh.
#
# ── What this script OWNS ───────────────────────────────────────────────
#   /home/unitree/robotics-connect/                (rsynced code tree)
#   ~/miniconda3/envs/robotics-connect/            (dedicated conda env)
#   /etc/systemd/system/robotics-connect-*.service    (our services only)
#   /etc/profile.d/robotics-connect.sh             (activation hook)
#
# ── What this script DOES NOT TOUCH (factory artifacts) ────────────────
#   /home/unitree/cyclonedds_ws/              (factory DDS workspace)
#   /home/unitree/unitree_ros2/               (factory ROS2 overlay)
#   /home/unitree/unitree_sdk2_python/        (factory Python SDK)
#   /home/unitree/unitree_sdk2-main/          (factory C++ SDK)
#   /home/unitree/miniconda3/envs/unitree_deploy/  (factory env — we clone, not mutate)
#   /home/unitree/wifi-bt-deb/                (factory driver package)
#   /home/unitree/librealsense/               (factory camera lib)
#   /opt/ros/foxy/                            (factory ROS distro)
#   /etc/systemd/system/nv*.service           (factory NV init)
#   ~/.bashrc                                 (we source a separate profile.d hook)
#
# Invariants:
#   - idempotent: safe to re-run any number of times,
#   - reversible: ./uninstall.sh returns the host to a byte-for-byte
#     factory state, modulo conda package caches,
#   - loud on divergence: if a factory artifact is missing, we fail
#     with a clear diagnostic rather than attempt to heal it.

set -euo pipefail

# ── Paths ───────────────────────────────────────────────────────────────

INSTALL_DIR="/home/unitree/robotics-connect"
CONDA_ENV="robotics-connect"
PROFILE_HOOK="/etc/profile.d/robotics-connect.sh"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${HERE}/.." && pwd)"

# ── --verify / verify: run the install self-check instead of installing ──
# Folds the by-hand sensor + hand checks into one PASS/FAIL scoreboard (see verify.sh).
if [[ "${1:-}" == "--verify" || "${1:-}" == "verify" ]]; then
    shift
    exec bash "${HERE}/verify.sh" "$@"
fi

# Offline bundle layout: produced by build-offline-bundle.sh on a machine
# with internet + docker, then shipped to the robot. If a sibling offline/
# directory exists (i.e. the repo was extracted from robotics-connect-offline-*.tar),
# install.sh consumes it automatically — no internet needed on the robot.
#
#   <bundle-root>/
#   ├── source/                              → this becomes REPO_ROOT
#   └── offline/
#       ├── robotics-connect-vision-sidecar.tar      → docker load
#       └── wheels/                          → pip --find-links --no-index
OFFLINE_DIR="${OFFLINE_DIR:-${REPO_ROOT}/../offline}"
[[ -d "${OFFLINE_DIR}" ]] || OFFLINE_DIR=""

# ── Logging ─────────────────────────────────────────────────────────────

say() { printf '\033[1;36m[install]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[install]\033[0m %s\n' "$*" >&2; }
die() { printf '\033[1;31m[install]\033[0m %s\n' "$*" >&2; exit 1; }

if [[ "${EUID}" -eq 0 ]]; then
    SUDO=""
else
    SUDO="sudo -E "
fi

# ── Step 1. Preflight: are we on a G1 EDU with factory stack intact? ───
# We refuse to run on anything that doesn't look like a factory G1 EDU.
# The purpose is to avoid wrecking a non-target machine by mistake.

say "Preflight checks"

[[ "$(uname -m)" == "aarch64" ]] || die "This is not aarch64 — are you on a G1 EDU?"
[[ "$(uname -r)" == *tegra* ]]  || die "This is not a Tegra kernel — are you on a G1 EDU?"
[[ -d /etc/nvpmodel          ]] || die "No /etc/nvpmodel — this does not look like a Jetson."

required_factory_paths=(
    /home/unitree/cyclonedds_ws/install/setup.bash
    /home/unitree/unitree_sdk2_python/setup.py
    /home/unitree/unitree_ros2
    /home/unitree/miniconda3/etc/profile.d/conda.sh
    /opt/ros/foxy/setup.bash
)
for p in "${required_factory_paths[@]}"; do
    [[ -e "$p" ]] || die "Missing factory artifact: $p — is this G1 EDU Type F stock?"
done

# ── Step 2. WiFi persistence (optional; see setup-wifi.sh) ──────────────
# The factory ships with WiFi driver NOT loaded. If the user hasn't run
# setup-wifi.sh yet we don't force it here — they might be installing over
# ethernet on purpose. Just report state.

if systemctl is-enabled --quiet nvwifibt.service 2>/dev/null \
   && ip -br link show wlan0 2>/dev/null | grep -q 'UP'; then
    say "WiFi is persistent (nvwifibt.service active, wlan0 UP) — good"
else
    warn "WiFi is not persistently enabled. Run ./setup-wifi.sh to enable it"
    warn "(not blocking — install over ethernet is fine)"
fi

# ── Step 3. Deploy code tree to /home/unitree/robotics-connect/ ──────────────
# rsync from the repo checkout. Exclude build/test/demo artifacts.

say "Deploying code to ${INSTALL_DIR}"
mkdir -p "${INSTALL_DIR}"
# install/ IS deployed — operators need uninstall.sh on the robot for rollback,
# and having setup-wifi.sh / first-boot.sh on-robot is useful for re-running
# without re-scp'ing the repo.
rsync -a --delete \
    --exclude='__pycache__/' \
    --exclude='*.pyc' \
    --exclude='.pytest_cache/' \
    --exclude='.git/' \
    "${REPO_ROOT}/" "${INSTALL_DIR}/"

# ── Step 4. Dedicated conda env: robotics-connect ────────────────────────────
# We do NOT mutate `unitree_deploy` (the factory env). Instead we clone it
# once and then layer the Robotics Connect requirements on top. `conda create --clone`
# gets us a starting point with all factory packages already pinned.
#
# Rationale: the factory env is a minefield of ad-hoc `pip install` history
# that has no requirements file. Cloning captures that state byte-for-byte.

# shellcheck source=/dev/null
source /home/unitree/miniconda3/etc/profile.d/conda.sh

if conda env list | awk '{print $1}' | grep -qx "${CONDA_ENV}"; then
    say "Conda env '${CONDA_ENV}' already exists — skipping create"
else
    say "Cloning unitree_deploy → ${CONDA_ENV} (takes 2–5 min)"
    conda create --clone unitree_deploy --name "${CONDA_ENV}" --yes
fi

say "Applying Robotics Connect requirements delta"
# shellcheck disable=SC1091
conda activate "${CONDA_ENV}"
pip_args=(--no-deps --upgrade -r "${HERE}/requirements.txt")
if [[ -n "${OFFLINE_DIR}" && -d "${OFFLINE_DIR}/wheels" ]]; then
    say "  (using offline wheels from ${OFFLINE_DIR}/wheels)"
    pip_args+=(--no-index --find-links "${OFFLINE_DIR}/wheels")
fi
pip install "${pip_args[@]}"
conda deactivate

# ── Step 5. Vision sidecar — OPT-IN ─────────────────────────────────────
# The sidecar runs DINOv2 for GPU embedding workloads.
# It is only needed for those workloads, and the image weighs ~12 GB.
# Default: skip. Set WITH_SIDECAR=1 to include it.

if [[ "${WITH_SIDECAR:-0}" == "1" ]]; then
    if [[ -n "${OFFLINE_DIR}" && -f "${OFFLINE_DIR}/robotics-connect-vision-sidecar.tar" ]]; then
        say "Staging offline sidecar image → /tmp/robotics-connect-vision-sidecar.tar"
        cp -n "${OFFLINE_DIR}/robotics-connect-vision-sidecar.tar" /tmp/robotics-connect-vision-sidecar.tar
    fi
    say "Installing vision sidecar (WITH_SIDECAR=1)"
    (cd "${INSTALL_DIR}/vision_sidecar" && ./install.sh)
else
    say "Skipping vision sidecar (set WITH_SIDECAR=1 to include it — optional GPU embedding sidecar)"
fi

# ── Step 6. Profile.d activation hook ───────────────────────────────────
# A single idempotent script that any shell on the robot can `source` to
# enter the Robotics Connect env. We install to /etc/profile.d so it's picked up
# automatically on login shells, but the hook also works standalone.

say "Installing shell activation hook to ${PROFILE_HOOK}"
${SUDO}tee "${PROFILE_HOOK}" >/dev/null <<'EOF'
# robotics-connect activation hook — installed by robotics-connect/install/install.sh.
# Remove with robotics-connect/install/uninstall.sh. Sourced by login shells.
if [[ -z "${ROBOTICS_CONNECT_ACTIVATED:-}" ]]; then
    export ROBOTICS_CONNECT_ACTIVATED=1
    export ROBOTICS_CONNECT_HOME=/home/unitree/robotics-connect
    export CYCLONEDDS_HOME=/home/unitree/cyclonedds_ws/install/cyclonedds
    export CYCLONEDDS_URI=/home/unitree/cyclonedds_ws/cyclonedds.xml
    export PATH="${ROBOTICS_CONNECT_HOME}/bin:${PATH}"
fi
EOF
${SUDO}chmod 644 "${PROFILE_HOOK}"

# ── Done ────────────────────────────────────────────────────────────────

say "Robotics Connect installed. Next steps:"
echo "  1. Open a new shell (or source ${PROFILE_HOOK})"
echo "  2. conda activate ${CONDA_ENV}"
echo "  3. python ${INSTALL_DIR}/arm_fk/arm_fk.py"
echo
echo "  Verify the whole install in one command:  bash ${INSTALL_DIR}/install/install.sh --verify"
echo "  Uninstall:                                ${INSTALL_DIR}/install/uninstall.sh"
