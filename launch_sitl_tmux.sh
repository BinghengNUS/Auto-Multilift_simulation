#!/usr/bin/env bash
# single-window tmux launch (6 panes)

set -eo pipefail
SESSION=${1:-sitl}
DIR="$(cd "$(dirname "$0")" && pwd)"
SIM="$HOME/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh"
ENV="source $DIR/install/setup.bash"   # colcon env

# clean old session
tmux kill-session -t "$SESSION" 2>/dev/null || true

# use DISPLAY 0
export DISPLAY=:0

# 1) Create session & layout
tmux new-session -d -s "$SESSION" -n main
P0=$(tmux display-message -p -t "$SESSION":main '#{pane_id}')        # left-top

tmux split-window -h -t "$P0"                                       # right-top
P1=$(tmux display-message -p -t "$SESSION":main '#{pane_id}')

tmux select-pane -t "$P0"
tmux split-window -v -t "$P0"                                       # left-bottom
P2=$(tmux display-message -p -t "$SESSION":main '#{pane_id}')

tmux select-pane -t "$P1"
tmux split-window -v -t "$P1"                                       # right-mid
P3=$(tmux display-message -p -t "$SESSION":main '#{pane_id}')

tmux select-pane -t "$P1"
tmux split-window -v -t "$P1"                                       # right-bottom
P4=$(tmux display-message -p -t "$SESSION":main '#{pane_id}')

# 2) Start baseline processes in fixed panes
tmux send-keys -t "$P0" "bash -lc '$ENV && ros2 run px4_tf tf_convert'" C-m
tmux send-keys -t "$P1" "bash -lc '$ENV && MicroXRCEAgent udp4 -p 8888'" C-m
tmux send-keys -t "$P2" "bash -lc '$ENV && \"$SIM\" \"$DIR/src/sitl_sim/sitl_sim/iris_modified_sitl.py\"'" C-m

# 3) Wait in this script’s shell for PX4 status to appear
# shellcheck disable=SC1090
$ENV
echo 'Waiting for /fmu/out/vehicle_status_v1 ...'
until ros2 topic list 2>/dev/null | grep -qE '^/fmu/out/vehicle_status_v1$'; do
  sleep 2
done

# 3a) Also wait for the RGB image topic, then create a new pane for the viewer
echo 'Waiting for /rgb ...'
until ros2 topic list 2>/dev/null | grep -qE '^/rgb$'; do
  sleep 2
done

# Create P5 from the left-bottom pane (P2) and run the viewer there
tmux select-pane -t "$P2"
tmux split-window -h -t "$P2" \
  "bash -lc '$ENV && ros2 run image_tools showimage --ros-args -r image:=/rgb'"
P5=$(tmux display-message -p -t "$SESSION":main '#{pane_id}')

# 4) Launch state machine BEFORE geom_multilift
tmux send-keys -t "$P3" "bash -lc '$ENV && ros2 launch offboard_state_machine multi_drone_goto.launch.py'" C-m
sleep 10  # give it time to take off


tmux send-keys -t "$P4" "bash -lc '$ENV && ros2 run px4_offboard geom_multilift'" C-m

# 5) Layout & attach
tmux select-layout -t "$SESSION":main tiled
tmux attach -t "$SESSION"
