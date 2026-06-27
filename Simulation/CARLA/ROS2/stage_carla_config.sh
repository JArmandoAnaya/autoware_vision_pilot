#!/usr/bin/env bash
# Stage the CARLA run config into VisionPilot/config/ so the VisionPilot binary — which
# reads config/vision_pilot*.conf and config/homography_C_matrix.yaml relative to its CWD —
# drives the CARLA ego. The tracked VisionPilot defaults are NOT modified in git; this only
# installs the CARLA variants at run time (restore with `git checkout` if needed).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"          # .../Simulation/CARLA/ROS2
VP_CONFIG="$(cd "$HERE/../../../VisionPilot/config" && pwd)"

cp "$HERE/config/visionpilot.carla.conf"      "$VP_CONFIG/vision_pilot.conf"
cp "$HERE/config/visionpilot_ros2.carla.conf" "$VP_CONFIG/vision_pilot_ros2.conf"
cp "$HERE/config/homography_C_matrix.yaml"    "$VP_CONFIG/homography_C_matrix.yaml"
echo "[stage] CARLA config installed into $VP_CONFIG"
echo "        vision_pilot.conf · vision_pilot_ros2.conf · homography_C_matrix.yaml"
