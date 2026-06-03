# Sourceable env bootstrap for Depth Camera Sight.
#
# Usage:
#   source robotics-connect/depth_camera_sight/setup_env.sh
#
# This script is idempotent — sourcing it twice is safe.  It sets the
# runtime env vars that the in-process pyrealsense2 build and the
# unitree_sdk2py DDS stack need on the Unitree G1 EDU robot.
#
# If you are deploying to a customer robot with a different install
# prefix, override LIBRS_PREFIX before sourcing:
#
#   LIBRS_PREFIX=/opt/robotics-connect/vendor/librealsense \
#   source robotics-connect/depth_camera_sight/setup_env.sh

LIBRS_PREFIX="${LIBRS_PREFIX:-/home/unitree/librealsense}"

if [[ ! -d "$LIBRS_PREFIX/build" ]]; then
    echo "depth_camera_sight/setup_env.sh: $LIBRS_PREFIX/build not found — " \
         "run install_pyrealsense2.sh first." >&2
    return 1 2>/dev/null || exit 1
fi

_libp="$LIBRS_PREFIX/build"
_pyp="$LIBRS_PREFIX/build/wrappers/python"

case ":${LD_LIBRARY_PATH:-}:" in
    *":${_libp}:"*) ;;
    *) export LD_LIBRARY_PATH="${_libp}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ;;
esac

case ":${PYTHONPATH:-}:" in
    *":${_pyp}:"*) ;;
    *) export PYTHONPATH="${_pyp}${PYTHONPATH:+:$PYTHONPATH}" ;;
esac

# Unitree DDS / OpenMP runtime needed for the DDS video client to not crash
# with a libgomp symbol resolution error under conda.
if [[ -f "/home/unitree/miniconda3/envs/unitree_deploy/lib/libgomp.so.1" ]]; then
    export LD_PRELOAD="/home/unitree/miniconda3/envs/unitree_deploy/lib/libgomp.so.1${LD_PRELOAD:+:$LD_PRELOAD}"
fi

unset _libp _pyp
