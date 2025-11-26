#!/usr/bin/env bash
# Fast tmux launcher: Isaac Sim -> WAIT topics -> State machine -> Offboard

set -euo pipefail

############ CONFIG ############
SESSION=${1:-sitl}
DIR="$(cd "$(dirname "$0")" && pwd)"
SIM="$HOME/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$DIR/install/setup.bash"

TOPIC_PX4_STATUS="/fmu/out/vehicle_status_v1"
TOPIC_RGB="/rgb"
VIEWER_CMD="ros2 run image_tools showimage --ros-args -r image:=$TOPIC_RGB"

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export DISPLAY="${DISPLAY:-:0}"

HEADLESS_FLAGS=( "--no-window" "--/renderer/multiGpu/active=false" "--/app/file/ignoreUSDMTL=true" )
POLL_INTERVAL=0.2
TIMEOUT_SEC=300
########## /CONFIG ############

# Safe source under set -u
safe_source() {
  set +u
  export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

# Load env here (for waits)
safe_source "$ROS_SETUP"
[ -f "$WS_SETUP" ] && safe_source "$WS_SETUP"

# One-shot env wrapper for panes (no repeated source)
WITH_ENV="$DIR/.with_env.sh"
cat > "$WITH_ENV" <<'EOF'
#!/usr/bin/env bash
set -eo pipefail
set +u
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
source "__ROS_SETUP__"
[ -f "__WS_SETUP__" ] && source "__WS_SETUP__"
set -u
exec "$@"
EOF
sed -i "s|__ROS_SETUP__|$ROS_SETUP|g" "$WITH_ENV"
sed -i "s|__WS_SETUP__|$WS_SETUP|g" "$WITH_ENV"
chmod +x "$WITH_ENV"

now() { date '+%H:%M:%S'; }
wait_topic() {
  local t="$1" start elapsed; start=$(date +%s)
  echo "[$(now)] wait: $t"
  while ! ros2 topic info "$t" >/dev/null 2>&1; do
    sleep "$POLL_INTERVAL"
    if (( TIMEOUT_SEC > 0 )); then
      elapsed=$(( $(date +%s) - start ))
      (( elapsed >= TIMEOUT_SEC )) && { echo "[$(now)] timeout: $t" >&2; return 1; }
    fi
  done
  echo "[$(now)] ready: $t"
}

# Clean old session
tmux kill-session -t "$SESSION" 2>/dev/null || true

# Layout (6 panes)
tmux new-session -d -s "$SESSION" -n main
P0=$(tmux display-message -p -t "$SESSION":main '#{pane_id}')
tmux split-window -h -t "$P0";  P1=$(tmux display-message -p -t "$SESSION":main '#{pane_id}')
tmux select-pane -t "$P0"; tmux split-window -v -t "$P0";  P2=$(tmux display-message -p -t "$SESSION":main '#{pane_id}')
tmux select-pane -t "$P1"; tmux split-window -v -t "$P1";  P3=$(tmux display-message -p -t "$SESSION":main '#{pane_id}')
tmux select-pane -t "$P1"; tmux split-window -v -t "$P1";  P4=$(tmux display-message -p -t "$SESSION":main '#{pane_id}')

# Start core processes
tmux send-keys -t "$P0" "$WITH_ENV ros2 run px4_tf tf_convert" C-m
tmux send-keys -t "$P1" "$WITH_ENV MicroXRCEAgent udp4 -p 8888" C-m
ISAAC_CMD=( "$SIM" "${HEADLESS_FLAGS[@]}" "$DIR/src/sitl_sim/sitl_sim/iris_modified_sitl.py" )
tmux send-keys -t "$P2" "$WITH_ENV ${ISAAC_CMD[*]}" C-m

# >>> WAIT AFTER ISAAC SIM, BEFORE STATE MACHINE <<<
wait_topic "$TOPIC_PX4_STATUS"
wait_topic "$TOPIC_RGB"

# Viewer (from P2)
tmux select-pane -t "$P2"
tmux split-window -h -t "$P2" "$WITH_ENV $VIEWER_CMD"

# Launch state machine then offboard
tmux send-keys -t "$P3" "$WITH_ENV ros2 launch offboard_state_machine multi_drone_goto.launch.py" C-m
tmux send-keys -t "$P4" "$WITH_ENV ros2 run px4_offboard geom_multilift" C-m

tmux select-layout -t "$SESSION":main tiled
tmux attach -t "$SESSION"
