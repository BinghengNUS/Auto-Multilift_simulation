#!/usr/bin/env bash
set -eo pipefail
set +u
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
source "/opt/ros/humble/setup.bash"
[ -f "/home/carlson/ros2/multilift_ws/install/setup.bash" ] && source "/home/carlson/ros2/multilift_ws/install/setup.bash"
set -u
exec "$@"
