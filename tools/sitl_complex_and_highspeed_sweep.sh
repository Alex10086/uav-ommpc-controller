#!/bin/bash
# Run complex trajectories (circle, figure8, helix) and high-speed hover_to_hover
# under v<=4, a<=8, j<=20 constraints on SUPER, restarting PX4 before each flight.
set +u
VEHICLE="${VEHICLE:-super}"
PX4_TARGET="${PX4_TARGET:-gz_${VEHICLE}}"
LABEL="${LABEL:-super-orig}"
OUT_ROOT="${OUT_ROOT:-$HOME/uav_control_ws/ommpc-flight-results}"
COMPLEX_MODES="${COMPLEX_MODES-circle figure8 helix}"
COMPLEX_SPEEDS="${COMPLEX_SPEEDS-1.6 2.0}"
RADIUS="${RADIUS:-1.2}"
Z_AMP="${Z_AMP:-0.35}"
HIGHSPEEDS="${HIGHSPEEDS-2.5 3.0 3.5 4.0}"

restart_px4() {
  pkill -9 -x px4 2>/dev/null
  pkill -9 -f 'gz sim' 2>/dev/null
  sleep 2
  (
    source "$HOME/.venvs/px4-v1.17/bin/activate"
    export CMAKE_BUILD_PARALLEL_LEVEL=4 MAKEFLAGS=-j4
    cd "$HOME/PX4-Autopilot" || exit 1
    HEADLESS=1 nohup make px4_sitl "$PX4_TARGET" > /tmp/px4-sweep.log 2>&1 &
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
cd "$HOME/uav_control_ws/src/uav-ommpc-controller" || exit 1

for mode in $COMPLEX_MODES; do
  for speed in $COMPLEX_SPEEDS; do
    out_dir="$OUT_ROOT/${LABEL}-${mode}-${speed}ms"
    rm -rf "$out_dir"
    echo "=== label=${LABEL} mode=${mode} speed=${speed} radius=${RADIUS} z_amp=${Z_AMP} ==="
    restart_px4
    python3 tools/sitl_ommpc_test.py --confirm-sitl --vehicle "$VEHICLE" --mode "$mode" \
      --speed "$speed" --radius "$RADIUS" --z-amp "$Z_AMP" --loops 2.0 \
      --abort-tilt 1.0 --max-pos-rmse 1.0 --max-vel-rmse 1.0 \
      --output-dir "$out_dir" > "/tmp/run-${LABEL}-${mode}-${speed}.log" 2>&1
    rc=$?
    echo "=== label=${LABEL} mode=${mode} speed=${speed} exit=${rc} ==="
  done
done

for speed in $HIGHSPEEDS; do
  out_dir="$OUT_ROOT/${LABEL}-h2h-${speed}ms"
  rm -rf "$out_dir"
  echo "=== label=${LABEL} mode=hover_to_hover speed=${speed} offset=4.5 0 0 ==="
  restart_px4
  python3 tools/sitl_ommpc_test.py --confirm-sitl --vehicle "$VEHICLE" --mode hover_to_hover \
    --speed "$speed" --offset 4.5 0.0 0.0 \
    --abort-tilt 1.0 --max-pos-rmse 1.0 --max-vel-rmse 1.0 \
    --output-dir "$out_dir" > "/tmp/run-${LABEL}-h2h-${speed}.log" 2>&1
  rc=$?
  echo "=== label=${LABEL} mode=hover_to_hover speed=${speed} exit=${rc} ==="
done

pkill -9 -x px4 2>/dev/null
pkill -9 -f 'gz sim' 2>/dev/null
true
