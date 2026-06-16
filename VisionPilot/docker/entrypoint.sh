#!/usr/bin/env bash
# Source the ROS2 environment so rclcpp / message libraries are on the library
# path, then exec the requested command from the VisionPilot working directory
# (the binary reads build/config/homography_C_matrix.yaml relative to CWD).
set -e
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
cd /opt/visionpilot
exec "$@"
