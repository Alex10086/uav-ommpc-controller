#!/bin/bash
# Repeat SITL hover_to_hover runs at fixed speeds, restarting PX4 between runs so
# each starts landed and disarmed with the same initial condition.
#
#   SPEEDS='0.8 1.6 2.0' REPEATS=3 LABEL=ommpc bash tools/sitl_hover_to_hover_sweep.sh
#
# The controller under test runs separately (ommpc_node or tools/pd_so3_node.py);
# this script only publishes references and records evidence. Test-only: never
# real hardware.
set +u
SPEEDS="${SPEEDS:-0.4 0.8 1.2 1.6 2.0}"
OFFSET="${OFFSET:-2.0 0.0 0.0}"
REPEATS="${REPEATS:-1}"
LABEL="${LABEL:-sweep}"
OUT_ROOT="${OUT_ROOT:-$HOME/uav_control_ws/ommpc-flight-results}"

restart_px4() {
  pkill -9 -f 'bin/px4' 2>/dev/null
  pkill -9 -f 'gz sim' 2>/dev/null
  sleep 2
  (
    source "$HOME/.venvs/px4-v1.17/bin/activate"
    export CMAKE_BUILD_PARALLEL_LEVEL=4 MAKEFLAGS=-j4
    cd "$HOME/PX4-Autopilot" || exit 1
    HEADLESS=1 nohup make px4_sitl gz_x500 > /tmp/px4-sweep.log 2>&1 &
  )
  for _ in $(seq 1 90); do
    grep -q 'Ready for takeoff' /tmp/px4-sweep.log 2>/dev/null && break
    sleep 1
  done
  grep -q 'Ready for takeoff' /tmp/px4-sweep.log || { echo 'PX4 did not start'; exit 1; }
  sleep 5
}

source "$HOME/ros2-px4-env.bash"
source "$HOME/uav_control_ws/install/setup.bash"

for speed in $SPEEDS; do
  for run in $(seq 1 "$REPEATS"); do
    echo "=== label=${LABEL} speed=${speed} run=${run}/${REPEATS} offset=${OFFSET} ==="
    restart_px4
    cd "$HOME/uav_control_ws/src/uav-ommpc-controller" || exit 1
    python3 tools/sitl_ommpc_test.py --confirm-sitl --mode hover_to_hover \
      --speed "$speed" --offset $OFFSET --record-bag \
      --output-dir "$OUT_ROOT/${LABEL}-${speed}ms-r${run}" > /tmp/run-${LABEL}-${speed}-${run}.log 2>&1
    rc=$?
    echo "=== label=${LABEL} speed=${speed} run=${run} exit=${rc} ==="
  done
done
