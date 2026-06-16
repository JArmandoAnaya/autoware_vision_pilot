#!/usr/bin/env bash
# One command to drive VisionPilot on a running CARLA UE5 (--ros2) server.
#
# It does a clean teardown, then brings up — in the correct order — the three
# containers and the spawn helper, sharing the host network and the readiness
# handshake dir (/tmp/vp_ipc):
#
#   carla-odom-bridge      CARLA RPC odometry     -> /carla/hero/odometry
#   carla-control-bridge   VisionPilot ackermann  -> /carla/hero/vehicle_control_cmd
#   carla-spawn            spawns the ego + main_cam, holds it until VisionPilot ready
#   visionpilot-gpu        the driving binary (foreground; Ctrl-C stops everything)
#
# Prerequisites (build once with ./build_all.sh):
#   - images visionpilot-gpu, carla-odom-bridge, carla-control-bridge
#   - weights in VisionPilot/models/ (autodrive.onnx, autosteer_2.0.onnx, autospeed_2.onnx)
#   - a CARLA UE5 0.10 server already running with:
#       FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ./CarlaUnreal.sh --ros2
#
# The config is VisionPilot/config/vision_pilot_carla.conf.example by default
# (override with VP_CONFIG). The spawn scenario is config/VisionPilot_carla10.json
# (override with SPAWN_CONFIG).
set -euo pipefail

DOCKER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_DIR="$(cd "${DOCKER_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${DOCKER_DIR}/../../../.." && pwd)"

SPAWN_CONFIG="${SPAWN_CONFIG:-config/VisionPilot_carla10.json}"
VP_IPC_DIR="${VP_IPC_DIR:-/tmp/vp_ipc}"

# --- preflight --------------------------------------------------------------
for img in visionpilot-gpu carla-odom-bridge carla-control-bridge; do
    if ! docker image inspect "${img}" &>/dev/null; then
        echo "ERROR: image '${img}' not found. Run ./build_all.sh first."
        exit 1
    fi
done

if ! timeout 2 bash -c '</dev/tcp/127.0.0.1/2000' 2>/dev/null; then
    echo "ERROR: nothing is listening on CARLA RPC port 2000."
    echo "  Start CARLA first:  FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ./CarlaUnreal.sh --ros2"
    exit 1
fi

mkdir -p "${VP_IPC_DIR}"

# --- clean slate (stale DDS publishers from a previous run break a new one) -
"${DOCKER_DIR}/down.sh"
# Stop on Ctrl-C / exit: tear the whole stack down so nothing lingers.
trap '"${DOCKER_DIR}/down.sh"' EXIT

# The ego must exist (and the spawn helper's world setup must be done) BEFORE the
# bridges connect — a bridge that opens a world handle first gets it invalidated
# when the spawn helper configures the world, and dies. So: spawn first, wait for
# the ego, then start the bridges.
echo "==> spawn ego (held until VisionPilot is ready)"
# Reuse the odom-bridge image: it carries the CARLA cp310 Python client. Override
# its entrypoint to run the spawn helper from the mounted repo. It shares the
# readiness sentinel dir with the VisionPilot container.
docker run -d --rm --name carla-spawn \
    --net=host --ipc=host \
    -v "${ROS2_DIR}:/ws2" \
    -v "${VP_IPC_DIR}:/tmp/vp_ipc" \
    -w /ws2 \
    --entrypoint micromamba \
    carla-odom-bridge \
    run -p /opt/conda/envs/odom python ros_carla_config.py -f "${SPAWN_CONFIG}" >/dev/null

echo -n "    waiting for ego to spawn"
for _ in $(seq 1 30); do
    if docker logs carla-spawn 2>&1 | grep -qiE "Holding ego|ego loc="; then
        echo " — up."
        break
    fi
    if ! docker ps --format '{{.Names}}' | grep -q carla-spawn; then
        echo ""
        echo "ERROR: spawn helper exited before creating the ego:"
        docker logs carla-spawn 2>&1 | tail -15
        exit 1
    fi
    echo -n "."
    sleep 1
done

echo "==> odom bridge"
# --restart on-failure (not --rm; they conflict) self-heals the rare case where
# the rclpy context still gets torn down at startup. down.sh removes it by name.
docker run -d --restart on-failure:5 --name carla-odom-bridge \
    --net=host --ipc=host -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
    carla-odom-bridge >/dev/null

echo "==> control bridge"
docker run -d --rm --name carla-control-bridge \
    --net=host --ipc=host -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
    carla-control-bridge >/dev/null

echo "==> VisionPilot (Ctrl-C to stop everything)"
# Run in the foreground (not exec) so the EXIT trap fires and tears the stack down.
"${REPO_ROOT}/VisionPilot/docker/run.sh"
