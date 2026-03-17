#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

conda deactivate 2>/dev/null || true
# ROS2 setup scripts may reference unset variables; temporarily disable nounset.
set +u
source /opt/ros/humble/setup.bash
set -u
colcon build --packages-select glove telekinesis
