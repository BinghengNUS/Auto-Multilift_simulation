#!/usr/bin/env bash
# single-window tmux launch (4 panes)

set -eo pipefail                       # no “-u”
SESSION=${1:-sitl}
DIR="$(cd "$(dirname "$0")" && pwd)"
SIM="$HOME/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh"
ENV="source $DIR/install/setup.bash"   # colcon env

# clean old session
tmux kill-session -t "$SESSION" 2>/dev/null || true

# pane-0
tmux new -d -s "$SESSION" -n main \
  "bash -lc '$ENV && ros2 run px4_tf tf_convert'"

# pane-1
tmux split-window -h \
  "bash -lc '$ENV && MicroXRCEAgent udp4 -p 8888'"

# pane-2
tmux split-window -v -t "$SESSION":0.0 \
  "bash -lc '$ENV && $SIM $DIR/src/sitl_sim/sitl_sim/sitl_stable.py'"

# wait until PX4 status appears
$ENV
until ros2 topic echo /px4_5/fmu/out/vehicle_status_v1 --once >/dev/null 2>&1; do sleep 2; done

# pane-3
tmux split-window -v -t "$SESSION":0.1 \
  "bash -lc '$ENV && ros2 launch px4_offboard geom_multi.launch.py'"

tmux select-layout tiled              # 2×2 grid
tmux attach -t "$SESSION"             # show panes
