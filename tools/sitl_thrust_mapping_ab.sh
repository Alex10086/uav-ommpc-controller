#!/bin/bash
# A/B comparison of px4_bridge thrust mapping on SUPER with the OMMPC controller unchanged.
# Each flight restarts PX4 so every run starts landed and disarmed.
#   MAPPINGS='linear quadratic' REPEATS=3 bash tools/sitl_thrust_mapping_ab.sh
set +u
MAPPINGS="${MAPPINGS-linear quadratic}"
REPEATS="${REPEATS:-3}"
FIRST_RUN="${FIRST_RUN:-1}"
CASE_FILTER="${CASE_FILTER:-}"
OUT_ROOT="${OUT_ROOT:-$HOME/uav_control_ws/ommpc-flight-results}"
SRC="$HOME/uav_control_ws/src/uav-ommpc-controller"
OMMPC_CONFIG="$SRC/uav_ommpc_ros/config/ommpc_super_sitl.yaml"

# name | sitl_ommpc_test.py arguments
CASES=(
  "helix-3.0ms|--mode helix --speed 3.0 --radius 1.6 --z-amp 0.40 --loops 2.0"
  "sharp3d-3.85ms|--mode sharp3d --speed 3.85 --radius 1.65 --z-amp 0.60"
  "scurve-3.9ms|--mode scurve --speed 3.9 --radius 1.6 --loops 2.0"
  "h2h-4.0ms|--mode hover_to_hover --speed 4.0 --offset 4.5 0.0 0.0"
)

restart_px4() {
  pkill -9 -x px4 2>/dev/null
  pkill -9 -f 'gz sim' 2>/dev/null
  sleep 2
  (
    source "$HOME/.venvs/px4-v1.17/bin/activate"
    export CMAKE_BUILD_PARALLEL_LEVEL=4 MAKEFLAGS=-j4
    cd "$HOME/PX4-Autopilot" || exit 1
    nohup bash "$SRC/tools/start_super_sitl.sh" > /tmp/px4-sweep.log 2>&1 &
  )
  for _ in $(seq 1 90); do
    grep -q 'Ready for takeoff' /tmp/px4-sweep.log 2>/dev/null && break
    sleep 1
  done
  grep -q 'Ready for takeoff' /tmp/px4-sweep.log || { echo 'PX4 did not start'; exit 1; }
  sleep 5
}

start_nodes() {
  local quadratic="$1"
  tmux kill-window -t sitl:bridge 2>/dev/null
  tmux kill-window -t sitl:ommpc 2>/dev/null
  sleep 1
  tmux new-window -t sitl -n bridge "source ~/ros2-px4-env.bash && source ~/uav_control_ws/install/setup.bash && ros2 run uav_ommpc_ros px4_bridge --ros-args -p hover_thrust:=0.309128 -p quadratic_thrust_model:=${quadratic}"
  tmux new-window -t sitl -n ommpc "source ~/ros2-px4-env.bash && source ~/uav_control_ws/install/setup.bash && ros2 run uav_ommpc_ros ommpc_node --ros-args --params-file ${OMMPC_CONFIG}"
  sleep 3
}

source "$HOME/ros2-px4-env.bash"
source "$HOME/uav_control_ws/install/setup.bash"
cd "$SRC" || exit 1

for mapping in $MAPPINGS; do
  quadratic=false
  [ "$mapping" = "quadratic" ] && quadratic=true
  start_nodes "$quadratic"
  for case in "${CASES[@]}"; do
    name="${case%%|*}"
    args="${case#*|}"
    [ -n "$CASE_FILTER" ] && [ "$name" != "$CASE_FILTER" ] && continue
    for run in $(seq "$FIRST_RUN" "$((FIRST_RUN + REPEATS - 1))"); do
      out_dir="$OUT_ROOT/thrmap-${mapping}-${name}-r${run}"
      if [ -e "$out_dir" ]; then
        echo "Refusing to overwrite evidence: $out_dir" >&2
        exit 1
      fi
      echo "=== mapping=${mapping} case=${name} run=${run} batch_repeats=${REPEATS} ==="
      restart_px4
      python3 tools/sitl_ommpc_test.py --confirm-sitl --vehicle super $args \
        --abort-tilt 1.0 --max-pos-rmse 1.0 --max-vel-rmse 1.0 \
        --output-dir "$out_dir" > "/tmp/run-thrmap-${mapping}-${name}-r${run}.log" 2>&1
      echo "=== mapping=${mapping} case=${name} run=${run} exit=$? ==="
    done
  done
done

# Leave the default stack running: quadratic bridge + tuned OMMPC.
start_nodes true
pkill -9 -x px4 2>/dev/null
pkill -9 -f 'gz sim' 2>/dev/null
true
