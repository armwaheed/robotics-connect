#!/usr/bin/env bash
# Source this before running Isaac Sim / Isaac Lab on a DGX Spark (GB10, aarch64).
#
#   source spark_env.sh            # sets the aarch64 must-do env, defines `isaaclab`
#   isaaclab -p path/to/script.py --headless ...
#
# It captures the non-obvious aarch64/GB10 caveats that otherwise cost hours (see SKILL.md).
# Override ISAACLAB_DIR if your Isaac Lab checkout is elsewhere.

ISAACLAB_DIR="${ISAACLAB_DIR:-$HOME/workspaces/git/IsaacLab}"

# aarch64 MUST-DO: preload libgomp before every Isaac/Isaac Lab run, or it crashes on import.
_LIBGOMP="/lib/aarch64-linux-gnu/libgomp.so.1"
if [ -e "$_LIBGOMP" ]; then
    case ":$LD_PRELOAD:" in
        *":$_LIBGOMP:"*) ;;                              # already present
        *) export LD_PRELOAD="${LD_PRELOAD:+$LD_PRELOAD:}$_LIBGOMP" ;;
    esac
else
    echo "spark_env: WARNING — $_LIBGOMP not found; Isaac may crash without the libgomp preload." >&2
fi

# `isaaclab.sh` is a RELATIVE launcher — cd-ing into a subdir and running it gives a silent `exit 127`.
# This wrapper always invokes it from the IsaacLab dir while keeping your current working directory,
# so absolute -p script paths work from anywhere.
isaaclab() {
    if [ ! -x "$ISAACLAB_DIR/isaaclab.sh" ]; then
        echo "isaaclab: $ISAACLAB_DIR/isaaclab.sh not found (set ISAACLAB_DIR)." >&2
        return 1
    fi
    ( cd "$ISAACLAB_DIR" && ./isaaclab.sh "$@" )
}

echo "spark_env: LD_PRELOAD set; \`isaaclab\` runs from $ISAACLAB_DIR. GB10 is sm_121 — torch warns but runs."
