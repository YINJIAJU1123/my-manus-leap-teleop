#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
colcon build --packages-select glove telekinesis
