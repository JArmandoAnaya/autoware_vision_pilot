#!/usr/bin/env bash
# Run the visionpilot-gpu container against a running CARLA (--ros2) server.
# Only runtime host dep is the NVIDIA driver (CUDA/cuDNN/ORT are in the image).
# The model cache is bind-mounted so ORT's first-inference JIT (PTX->sm_120) is
# reused across runs for faster startup.
# Env: VP_CONFIG, VP_WEIGHTS, VP_MODEL_CACHE, VP_IPC_DIR, IMAGE_TAG, DISPLAY.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VP_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
IMAGE_NAME="visionpilot-gpu:${IMAGE_TAG:-latest}"

VP_CONFIG="${VP_CONFIG:-${VP_DIR}/config/vision_pilot_carla.conf.example}"
VP_WEIGHTS="${VP_WEIGHTS:-${VP_DIR}/models}"
VP_MODEL_CACHE="${VP_MODEL_CACHE:-${HOME}/.cache/visionpilot/model_cache}"
# Readiness handshake dir (control.ready_sentinel_path). Shared with the spawn
# helper so the ego is held stationary until VisionPilot's first command.
VP_IPC_DIR="${VP_IPC_DIR:-/tmp/vp_ipc}"
CONTAINER_NAME="${CONTAINER_NAME:-visionpilot-gpu}"
DISPLAY="${DISPLAY:-:1}"

if ! docker image inspect "${IMAGE_NAME}" &>/dev/null; then
    echo "ERROR: Image '${IMAGE_NAME}' not found. Run './build.sh' first."
    exit 1
fi
if [[ ! -f ${VP_CONFIG} ]]; then
    echo "ERROR: config not found: ${VP_CONFIG} (set VP_CONFIG=/path/to/conf)"
    exit 1
fi

# Shared ONNX/CUDA JIT cache + readiness-handshake dir (created if missing).
mkdir -p "${VP_MODEL_CACHE}" "${VP_IPC_DIR}"

WEIGHTS_MOUNT=()
if [[ -d ${VP_WEIGHTS} ]]; then
    WEIGHTS_MOUNT=(-v "${VP_WEIGHTS}:/weights:ro")
fi

echo "Starting ${IMAGE_NAME} (model cache: ${VP_MODEL_CACHE})..."
docker run --rm \
    --name "${CONTAINER_NAME}" \
    --gpus all \
    --net=host \
    --ipc=host \
    -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
    -e DISPLAY="${DISPLAY}" \
    -e CUDA_CACHE_PATH=/model_cache \
    -e CUDA_CACHE_MAXSIZE=2147483648 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "${VP_MODEL_CACHE}:/model_cache" \
    -v "${VP_IPC_DIR}:/tmp/vp_ipc" \
    -v "${VP_CONFIG}:/tmp/vp.conf:ro" \
    "${WEIGHTS_MOUNT[@]}" \
    "${IMAGE_NAME}" \
    ./build/VisionPilot --config /tmp/vp.conf
