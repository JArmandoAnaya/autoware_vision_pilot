#!/usr/bin/env bash
# Build all three images needed for the CARLA UE5 camera-only drive, once:
#
#   visionpilot-gpu        the single-binary driving system (CUDA + ONNX Runtime)
#   carla-odom-bridge      CARLA RPC odometry  -> nav_msgs/Odometry
#   carla-control-bridge   VisionPilot ackermann -> carla_msgs/CarlaEgoVehicleControl
#
# Prerequisite: the CARLA Python 3.10 wheel (for the odom bridge). Download it
# from the CARLA releases page and export its path:
#   export CARLA_WHL=/path/to/carla-0.10.0-cp310-cp310-linux_x86_64.whl
#
# Usage:
#   export CARLA_WHL=/path/to/carla-...whl
#   ./build_all.sh
set -euo pipefail

DOCKER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${DOCKER_DIR}/../../../.." && pwd)"

echo "==> [1/3] visionpilot-gpu"
"${REPO_ROOT}/VisionPilot/docker/build.sh"

echo "==> [2/3] carla-odom-bridge"
"${DOCKER_DIR}/odom_bridge/build.sh"

echo "==> [3/3] carla-control-bridge"
"${DOCKER_DIR}/control_bridge/build.sh"

echo ""
echo "All images built:"
docker image ls --format '  {{.Repository}}:{{.Tag}}' |
    grep -E 'visionpilot-gpu|carla-odom-bridge|carla-control-bridge' || true
