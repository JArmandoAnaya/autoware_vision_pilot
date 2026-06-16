#!/usr/bin/env bash
# Tear down every container started by drive.sh (does NOT touch the CARLA server,
# which you run yourself). Run this between experiments — stale ROS2/DDS publishers
# from a previous run interpolate with a new one. Safe to run anytime.
set -uo pipefail

for c in visionpilot-gpu carla-spawn carla-control-bridge carla-odom-bridge; do
    docker rm -f "${c}" >/dev/null 2>&1 && echo "removed ${c}" || true
done
echo "teardown complete"
