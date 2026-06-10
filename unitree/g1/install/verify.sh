#!/usr/bin/env bash
#
# verify.sh — Self-verify a Robotics Connect install on a Unitree G1 EDU in ONE command.
#
# Folds the sensor + hand checks that used to be run by hand into a single PASS/FAIL scoreboard,
# so "is this install good?" is provable on any G1 without a human running ~10 ad-hoc checks.
# Pure software + read-only sensor reads — NO robot motion.
#
# Usage:
#   install.sh --verify                 # (wired entrypoint) or:
#   bash install/verify.sh              # run all checks
#   bash install/verify.sh arm_fk hands # run a subset
#
# Exit 0 iff every selected check passes. Override env if your layout differs:
#   ROBOTICS_CONNECT_HOME (default /home/unitree/robotics-connect)
#   RC_CONDA_SH           (default /home/unitree/miniconda3/etc/profile.d/conda.sh)
#   RC_SENSOR_ENV         (default robotics-connect)   RC_HAND_ENV (default g1brainco)
#   CYCLONEDDS_URI        (default file:///home/unitree/cyclonedds.xml)

set -uo pipefail

RC_HOME="${ROBOTICS_CONNECT_HOME:-/home/unitree/robotics-connect}"
CONDA_SH="${RC_CONDA_SH:-/home/unitree/miniconda3/etc/profile.d/conda.sh}"
SENSOR_ENV="${RC_SENSOR_ENV:-robotics-connect}"
HAND_ENV="${RC_HAND_ENV:-g1brainco}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file:///home/unitree/cyclonedds.xml}"

c_pass=$'\033[1;32m'; c_fail=$'\033[1;31m'; c_dim=$'\033[2m'; c_off=$'\033[0m'

NAMES=(); RESULTS=(); NOTES=()
record() { NAMES+=("$1"); RESULTS+=("$2"); NOTES+=("$3"); }

# run_in_env <env> <command...> — run a command in a conda env, echo combined output, return its rc.
run_in_env() {
    local env="$1"; shift
    ( source "$CONDA_SH" 2>/dev/null && conda activate "$env" 2>/dev/null
      source "$RC_HOME/depth_camera_sight/setup_env.sh" 2>/dev/null || true
      "$@" 2>&1 )
}

# check <name> <env> <sentinel-regex> <command...> — PASS iff rc==0 AND output matches the sentinel.
check() {
    local name="$1" env="$2" sentinel="$3"; shift 3
    local out rc
    out="$(run_in_env "$env" "$@")"; rc=$?
    if [[ $rc -eq 0 ]] && grep -qE "$sentinel" <<<"$out"; then
        record "$name" PASS "$(grep -m1 -E "$sentinel" <<<"$out" | sed 's/^[[:space:]]*//')"
    else
        record "$name" FAIL "rc=$rc; $(tail -n1 <<<"$out" | sed 's/^[[:space:]]*//')"
    fi
}

# ── individual checks ───────────────────────────────────────────────────────
check_install() {
    local notes=() ok=1
    [[ -d "$RC_HOME" ]] || { ok=0; notes+=("missing $RC_HOME"); }
    [[ -f "$CONDA_SH" ]] || { ok=0; notes+=("no conda.sh"); }
    if source "$CONDA_SH" 2>/dev/null && conda env list 2>/dev/null | awk '{print $1}' | grep -qx "$SENSOR_ENV"; then :; else
        ok=0; notes+=("no '$SENSOR_ENV' env"); fi
    [[ -f /etc/profile.d/robotics-connect.sh ]] || notes+=("no profile hook (non-fatal)")
    if [[ $ok -eq 1 ]]; then record "install" PASS "$RC_HOME + '$SENSOR_ENV' env present"
    else record "install" FAIL "${notes[*]}"; fi
}
check_arm_fk() { check "arm_fk"  "$SENSOR_ENV" "SELFTEST_OK"               python "$RC_HOME/arm_fk/arm_fk.py"; }
check_camera() { check "camera"  "$SENSOR_ENV" "depth ok"                 timeout 30 python "$RC_HOME/depth_camera_sight/_diag_camera_test.py"; }
check_rgb()    { check "rgb"     "$SENSOR_ENV" "rgb[[:space:]]+ok"        timeout 30 python "$RC_HOME/depth_camera_sight/_diag_camera_test.py"; }
check_lidar()  { check "lidar"   "$SENSOR_ENV" "nearest_table_in_front|center=" timeout 30 python "$RC_HOME/lidar_sight/lidar_sight.py" --frames 5; }
check_hands()  { check "hands"   "$HAND_ENV"   "\"left\":.*\"right\":"    python "$RC_HOME/brainco_touch/brainco_bridge.py" --detect; }

# ── select + run ────────────────────────────────────────────────────────────
ALL=(install arm_fk camera rgb lidar hands)
SELECTED=("${@:-${ALL[@]}}")
[[ $# -eq 0 ]] && SELECTED=("${ALL[@]}")

printf '%s[verify]%s Robotics Connect self-check on %s\n' "$c_dim" "$c_off" "$(hostname)"
for c in "${SELECTED[@]}"; do
    case "$c" in
        install) check_install ;;
        arm_fk)  check_arm_fk ;;
        camera)  check_camera ;;
        rgb)     check_rgb ;;
        lidar)   check_lidar ;;
        hands)   check_hands ;;
        *) echo "[verify] unknown check: $c (known: ${ALL[*]})" >&2 ;;
    esac
done

# ── scoreboard ──────────────────────────────────────────────────────────────
echo
printf '  %-9s %-6s %s\n' "CHECK" "RESULT" "DETAIL"
printf '  %-9s %-6s %s\n' "-----" "------" "------"
fails=0
for i in "${!NAMES[@]}"; do
    r="${RESULTS[$i]}"
    if [[ "$r" == PASS ]]; then col="$c_pass"; else col="$c_fail"; ((fails++)); fi
    printf '  %-9s %b%-6s%b %s\n' "${NAMES[$i]}" "$col" "$r" "$c_off" "${NOTES[$i]}"
done
echo
if [[ $fails -eq 0 ]]; then
    printf '%s[verify] ALL %d CHECKS PASSED%s\n' "$c_pass" "${#NAMES[@]}" "$c_off"; exit 0
else
    printf '%s[verify] %d/%d CHECK(S) FAILED%s\n' "$c_fail" "$fails" "${#NAMES[@]}" "$c_off"; exit 1
fi
