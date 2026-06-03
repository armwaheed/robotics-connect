#!/usr/bin/env bash
# Build pyrealsense2 from source against the target Python.
#
# This script is idempotent: if a working build already exists at
# $INSTALL_PREFIX, it exits successfully without rebuilding.  Customer
# deploys of Robotics Connect should run this once per robot after cloning
# the repo — it's the only piece of `depth_camera_sight` that
# is platform-sensitive.
#
# Why is this necessary?
#
# The PyPI wheel `pyrealsense2` is built against GLIBC 2.32, but the
# stock Ubuntu 20.04 on the Unitree G1 EDU has 2.31.  It will import but
# immediately fail with `ImportError: GLIBC_2.32 not found`.  The
# apt-installed `ros-noetic-librealsense2` package ships the C++ library
# but no Python bindings.  The path that actually works is: build the
# librealsense Python bindings from source against the system glibc and
# the target conda Python interpreter.  That's what this script does.
#
# Requirements (all present out of the box on the G1 EDU's
# unitree_deploy install):
#
#   - gcc, g++ (system)
#   - cmake >= 3.8
#   - make
#   - libusb-1.0-0-dev
#   - libudev-dev
#   - A target Python 3.10 interpreter with the dev headers (conda envs
#     installed via miniconda satisfy this by default)
#   - Either (a) internet access to github.com to clone librealsense, or
#     (b) a pre-downloaded tarball at $SRC_TARBALL
#
# Usage:
#
#   # Default: build into /home/unitree/librealsense against unitree_deploy
#   bash install_pyrealsense2.sh
#
#   # Custom Python / prefix (for different robots or dev environments)
#   TARGET_PYTHON=/opt/conda/envs/myenv/bin/python \
#   INSTALL_PREFIX=$HOME/vendor/librealsense \
#   bash install_pyrealsense2.sh
#
# Output: sets up $INSTALL_PREFIX/build/ containing
#   - librealsense2.so.2.50.0    (core library)
#   - wrappers/python/pyrealsense2.cpython-310-aarch64-linux-gnu.so
# Consumers add these to LD_LIBRARY_PATH and PYTHONPATH — see
# `setup_env.sh` in this directory.

set -euo pipefail

# --- Configuration ----------------------------------------------------------

LIBREALSENSE_VERSION="${LIBREALSENSE_VERSION:-v2.50.0}"
INSTALL_PREFIX="${INSTALL_PREFIX:-$HOME/librealsense}"
TARGET_PYTHON="${TARGET_PYTHON:-/home/unitree/miniconda3/envs/unitree_deploy/bin/python}"
SRC_TARBALL="${SRC_TARBALL:-}"   # optional: offline path to a source tarball
BUILD_JOBS="${BUILD_JOBS:-4}"

echo "=== Robotics Connect pyrealsense2 installer ==="
echo "  librealsense version : $LIBREALSENSE_VERSION"
echo "  install prefix       : $INSTALL_PREFIX"
echo "  target python        : $TARGET_PYTHON"
echo "  parallel jobs        : $BUILD_JOBS"
echo

# --- Preflight --------------------------------------------------------------

if ! command -v cmake >/dev/null 2>&1; then
    echo "ERROR: cmake not found on PATH" >&2
    exit 1
fi
if ! command -v make >/dev/null 2>&1; then
    echo "ERROR: make not found on PATH" >&2
    exit 1
fi
if [[ ! -x "$TARGET_PYTHON" ]]; then
    echo "ERROR: TARGET_PYTHON ($TARGET_PYTHON) is not an executable" >&2
    exit 1
fi

PY_VERSION=$("$TARGET_PYTHON" -c 'import sys; print("{}.{}".format(sys.version_info[0], sys.version_info[1]))')
PY_ABITAG=$("$TARGET_PYTHON" -c 'import sysconfig; print(sysconfig.get_config_var("SOABI"))')
EXPECTED_SO="$INSTALL_PREFIX/build/wrappers/python/pyrealsense2.${PY_ABITAG}.so"

# --- Idempotence shortcut ---------------------------------------------------

if [[ -f "$EXPECTED_SO" ]]; then
    echo "Found existing build: $EXPECTED_SO"
    echo "Verifying import..."
    if LD_LIBRARY_PATH="$INSTALL_PREFIX/build:${LD_LIBRARY_PATH:-}" \
       PYTHONPATH="$INSTALL_PREFIX/build/wrappers/python:${PYTHONPATH:-}" \
       "$TARGET_PYTHON" -c "import pyrealsense2 as rs; print('  ok:', rs.__file__)" \
       >/dev/null 2>&1; then
        echo "  ok — import passes, nothing to do."
        exit 0
    else
        echo "  warn — existing build failed to import; will rebuild."
    fi
fi

# --- Fetch source ----------------------------------------------------------

mkdir -p "$INSTALL_PREFIX"
cd "$(dirname "$INSTALL_PREFIX")"

SRC_DIR="$INSTALL_PREFIX"
if [[ -d "$SRC_DIR/CMakeLists.txt" || -f "$SRC_DIR/CMakeLists.txt" ]]; then
    echo "Reusing existing source at $SRC_DIR"
elif [[ -n "$SRC_TARBALL" && -f "$SRC_TARBALL" ]]; then
    echo "Extracting offline source tarball: $SRC_TARBALL"
    rm -rf "$SRC_DIR"
    mkdir -p "$(dirname "$SRC_DIR")"
    tar -xzf "$SRC_TARBALL" -C "$(dirname "$SRC_DIR")"
    # Tarball may extract to 'librealsense' next to prefix — rename if so.
    if [[ -d "$(dirname "$SRC_DIR")/librealsense" && "$SRC_DIR" != "$(dirname "$SRC_DIR")/librealsense" ]]; then
        mv "$(dirname "$SRC_DIR")/librealsense" "$SRC_DIR"
    fi
else
    echo "Cloning librealsense $LIBREALSENSE_VERSION from github.com"
    rm -rf "$SRC_DIR"
    git clone -b "$LIBREALSENSE_VERSION" --depth 1 --recursive \
        https://github.com/IntelRealSense/librealsense.git "$SRC_DIR"
fi

# --- Configure + build ------------------------------------------------------

cd "$SRC_DIR"
mkdir -p build
cd build

echo
echo "=== cmake configure ==="
cmake .. \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_GRAPHICAL_EXAMPLES=OFF \
    -DBUILD_TOOLS=OFF \
    -DBUILD_UNIT_TESTS=OFF \
    -DBUILD_WITH_TM2=OFF \
    -DBUILD_PYTHON_BINDINGS=ON \
    -DPYTHON_EXECUTABLE="$TARGET_PYTHON" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCHECK_FOR_UPDATES=OFF

echo
echo "=== make -j$BUILD_JOBS (this takes ~10 min on aarch64) ==="
make -j"$BUILD_JOBS"

# --- Verify -----------------------------------------------------------------

if [[ ! -f "$EXPECTED_SO" ]]; then
    # Older builds may name the .so with a different ABI tag; find it.
    EXPECTED_SO=$(find "$SRC_DIR/build/wrappers/python" -name 'pyrealsense2*.so' | head -n 1)
fi

if [[ -z "${EXPECTED_SO:-}" || ! -f "$EXPECTED_SO" ]]; then
    echo "ERROR: build finished but no pyrealsense2*.so found" >&2
    exit 2
fi

echo
echo "=== verify import ==="
LD_LIBRARY_PATH="$INSTALL_PREFIX/build:${LD_LIBRARY_PATH:-}" \
PYTHONPATH="$INSTALL_PREFIX/build/wrappers/python:${PYTHONPATH:-}" \
"$TARGET_PYTHON" -c "import pyrealsense2 as rs; print('ok:', rs.__file__)"

echo
echo "=== done ==="
echo "Built: $EXPECTED_SO"
echo
echo "To use it, source the companion env script:"
echo "  source $(dirname "$(readlink -f "$0")")/setup_env.sh"
echo "or export these manually:"
echo "  export LD_LIBRARY_PATH=$INSTALL_PREFIX/build:\$LD_LIBRARY_PATH"
echo "  export PYTHONPATH=$INSTALL_PREFIX/build/wrappers/python:\$PYTHONPATH"
