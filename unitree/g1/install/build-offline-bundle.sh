#!/usr/bin/env bash
#
# build-offline-bundle.sh — Produce a self-contained bundle that installs
# Robotics Connect onto a Unitree G1 EDU with NO internet access on the robot.
#
# Output: ./dist/robotics-connect-offline-<git-sha>/ containing
#   source/                                 (git-tracked files only)
#   offline/robotics-connect-vision-sidecar.tar     (docker save output, ~12 GB; only when WITH_SIDECAR=1)
#   offline/wheels/                         (pip wheels for the requirements delta — empty today)
#   offline/wifi-bt-deb/                    (fallback Realtek WiFi driver, ~4 MB — optional)
#   INSTALL.md                              (operator-facing one-pager)
#
# Default bundle (no WITH_SIDECAR, no WiFi driver pulled): ~5 MB.
# WITH_SIDECAR=1: ~12 GB (sidecar image dominates).
#
# Plus dist/robotics-connect-offline-<sha>.tar — a single-file archive you
# can `scp` or copy onto a USB drive.
#
# Usage:
#
#   # With docker running and the sidecar image already built (or let
#   # this script build it from source). Run from the package root:
#   bash install/build-offline-bundle.sh
#
# Env vars:
#   SKIP_SIDECAR_BUILD=1    assume robotics-connect/vision-sidecar:0.1 already exists
#   SIDECAR_TAG=0.1         override sidecar image tag
#   WIFI_BT_DEB_SRC=<path>  directory containing rtl8852bu-dkms_*.deb +
#                           firmware/ + copy_firmware.sh to bundle as
#                           the setup-wifi.sh fallback. If unset, the
#                           script tries to scp it from WIFI_BT_DEB_ROBOT
#                           (default: unitree@192.168.123.164).
#   WIFI_BT_DEB_SKIP=1      don't bundle the WiFi driver at all (install
#                           relies on factory /home/unitree/wifi-bt-deb/)
#
# Run on your build machine (laptop/DGX/etc.), not on the robot.
# The robot only consumes the output.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${HERE}/.." && pwd)"   # the g1 package root (matches install.sh)

SIDECAR_TAG="${SIDECAR_TAG:-0.1}"
SIDECAR_IMAGE="robotics-connect/vision-sidecar:${SIDECAR_TAG}"

GIT_SHA="$(cd "${REPO_ROOT}" && git rev-parse --short HEAD 2>/dev/null || echo 'unknown')"
BUNDLE_NAME="robotics-connect-offline-${GIT_SHA}"
DIST="${REPO_ROOT}/dist"
BUNDLE_DIR="${DIST}/${BUNDLE_NAME}"

say() { printf '\033[1;36m[bundle]\033[0m %s\n' "$*"; }
die() { printf '\033[1;31m[bundle]\033[0m %s\n' "$*" >&2; exit 1; }

command -v docker >/dev/null 2>&1 || die "docker not on PATH"
command -v git    >/dev/null 2>&1 || die "git not on PATH"

say "Output dir: ${BUNDLE_DIR}"
rm -rf "${BUNDLE_DIR}"
mkdir -p "${BUNDLE_DIR}/source" "${BUNDLE_DIR}/offline/wheels"

# ── 1. Source: git archive honours .gitignore and excludes build cruft. ─

say "Packing package source tree via 'git archive'"
# REPO_ROOT is a subdirectory of the git repo (the g1 package); archive
# only that subtree so the bundle's source/ holds the package at its root.
PKG_TREEISH="HEAD"
PKG_PREFIX="$(cd "${REPO_ROOT}" && git rev-parse --show-prefix)"
[[ -n "${PKG_PREFIX}" ]] && PKG_TREEISH="HEAD:${PKG_PREFIX%/}"
(cd "${REPO_ROOT}" && git archive --format=tar "${PKG_TREEISH}") \
    | tar xf - -C "${BUNDLE_DIR}/source"

# ── 2. Sidecar image: build or reuse, then docker-save to tarball. ──────
# Opt-in via WITH_SIDECAR=1 (matches install.sh default). Operators who
# don't need the sidecar skip the 12 GB and get a ~5 MB bundle.

if [[ "${WITH_SIDECAR:-0}" == "1" ]]; then
    if [[ "${SKIP_SIDECAR_BUILD:-0}" != "1" ]]; then
        say "Building ${SIDECAR_IMAGE} from vision_sidecar/"
        docker build -t "${SIDECAR_IMAGE}" "${REPO_ROOT}/vision_sidecar"
    else
        docker image inspect "${SIDECAR_IMAGE}" >/dev/null 2>&1 \
            || die "${SIDECAR_IMAGE} not present and SKIP_SIDECAR_BUILD=1 — build first"
    fi
    say "Saving ${SIDECAR_IMAGE} → offline/robotics-connect-vision-sidecar.tar (~12 GB, takes a minute)"
    docker save "${SIDECAR_IMAGE}" -o "${BUNDLE_DIR}/offline/robotics-connect-vision-sidecar.tar"
else
    say "WITH_SIDECAR unset — skipping sidecar build and tarball (saves ~12 GB)"
fi

# ── 3. Wheels: today the requirements delta is empty (see requirements.txt). ─
# Kept as a no-op step so we can add deps later without restructuring.

if [[ -s "${HERE}/requirements.txt" ]] \
   && grep -q '^[a-zA-Z]' "${HERE}/requirements.txt"; then
    say "Downloading pip wheels from requirements.txt into offline/wheels/"
    pip download \
        --dest "${BUNDLE_DIR}/offline/wheels" \
        --no-deps \
        -r "${HERE}/requirements.txt" \
        --platform linux_aarch64 \
        --python-version 3.10 \
        --only-binary=:all:
else
    say "No non-empty requirements — skipping wheel download"
fi

# ── 4. WiFi driver fallback (optional) ──────────────────────────────────
# setup-wifi.sh searches factory → repo → offline → $WIFI_BT_DEB_DIR in
# order, so bundling the driver is a safety-net: it lets the robot set
# up WiFi even if Unitree ships a future image that strips
# /home/unitree/wifi-bt-deb/.

if [[ "${WIFI_BT_DEB_SKIP:-0}" == "1" ]]; then
    say "WIFI_BT_DEB_SKIP=1 — not bundling the WiFi driver"
elif [[ -n "${WIFI_BT_DEB_SRC:-}" && -d "${WIFI_BT_DEB_SRC}" ]]; then
    say "Copying WiFi driver from ${WIFI_BT_DEB_SRC} → offline/wifi-bt-deb/"
    cp -r "${WIFI_BT_DEB_SRC}/." "${BUNDLE_DIR}/offline/wifi-bt-deb/"
else
    robot="${WIFI_BT_DEB_ROBOT:-unitree@192.168.123.164}"
    if command -v sshpass >/dev/null 2>&1 && \
       sshpass -p 123 ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 "${robot}" \
            'test -d /home/unitree/wifi-bt-deb' 2>/dev/null; then
        say "Pulling WiFi driver from ${robot}:/home/unitree/wifi-bt-deb/"
        sshpass -p 123 scp -qr -o StrictHostKeyChecking=no \
            "${robot}:/home/unitree/wifi-bt-deb/." \
            "${BUNDLE_DIR}/offline/wifi-bt-deb/"
    else
        warn "No WiFi driver source available (set WIFI_BT_DEB_SRC or ensure ${robot} is reachable)"
        warn "— bundle will rely on factory /home/unitree/wifi-bt-deb/ on the target robot"
        rmdir "${BUNDLE_DIR}/offline/wifi-bt-deb" 2>/dev/null || true
    fi
fi
mkdir -p "${BUNDLE_DIR}/offline/wifi-bt-deb" 2>/dev/null || true  # idempotent; keep placeholder

# ── 5. Operator-facing one-pager ────────────────────────────────────────

cat > "${BUNDLE_DIR}/INSTALL.md" <<EOF
# Robotics Connect offline install — ${GIT_SHA}

You are holding a self-contained bundle that installs Robotics Connect onto
a Unitree G1 EDU **with no internet on the robot**.

## What's in this bundle

- \`source/\`                              Robotics Connect source (git sha ${GIT_SHA})
- \`offline/robotics-connect-vision-sidecar.tar\`  Pre-built GPU sidecar image (~12 GB)
- \`offline/wheels/\`                      Pip wheels for the requirements delta

## How to install

1. Copy this bundle onto the robot. Options:
   - USB drive → plug into the G1 → \`cp -r /media/unitree/*/robotics-connect-offline-*/ /tmp/\`
   - Ethernet from your laptop → \`scp -r ./robotics-connect-offline-${GIT_SHA} unitree@192.168.123.164:/tmp/\`

2. On the robot:
   \`\`\`
   cd /tmp/robotics-connect-offline-${GIT_SHA}/source
   sudo cp offline/robotics-connect-vision-sidecar.tar /tmp/   # where sidecar install.sh looks
   bash install/install.sh
   \`\`\`

3. Uninstall at any time:
   \`\`\`
   bash /home/unitree/robotics-connect/install/uninstall.sh
   \`\`\`

See \`install/README.md\` for the full deployment guide.
EOF

# ── 6. Single-file archive for easy scp/USB copy ────────────────────────

say "Tarring bundle to ${DIST}/${BUNDLE_NAME}.tar"
tar cf "${DIST}/${BUNDLE_NAME}.tar" -C "${DIST}" "${BUNDLE_NAME}"

say "Done. Bundle size:"
du -sh "${BUNDLE_DIR}" "${DIST}/${BUNDLE_NAME}.tar" | sed 's/^/  /'

say "Next: copy ${DIST}/${BUNDLE_NAME}.tar onto a USB drive or scp it to the robot."
