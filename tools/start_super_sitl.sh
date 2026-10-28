#!/usr/bin/env bash
# SUPER/Gazebo only: its GPS bridge has no 110 ms measurement-delay queue.
# Optional 110 is for an explicit historical comparison, not normal operation.
set -euo pipefail
if (( $# > 1 )); then
  echo "Usage: bash tools/start_super_sitl.sh [0|110]" >&2
  exit 2
fi
gps_delay_ms="${1:-0}"
case "$gps_delay_ms" in
  0|110) ;;
  *) echo "GPS delay must be 0 ms (normal) or 110 ms (historical comparison)" >&2; exit 2 ;;
esac
# Override even a stale saved parameter or inherited PX4_PARAM_* value.
export PX4_PARAM_EKF2_GPS_DELAY="$gps_delay_ms"
source "$HOME/.venvs/px4-v1.17/bin/activate"
cd "$HOME/PX4-Autopilot"
echo "SUPER SITL: EKF2_GPS_DELAY=${gps_delay_ms} ms"
HEADLESS=1 exec make -j4 px4_sitl gz_super
