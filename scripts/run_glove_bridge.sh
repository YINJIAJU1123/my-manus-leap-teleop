#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

conda deactivate 2>/dev/null || true
# ROS2 setup scripts may reference unset variables; temporarily disable nounset.
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
ros2 run glove read_and_send_zmq
