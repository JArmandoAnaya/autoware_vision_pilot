#!/usr/bin/env bash
# Build the visionpilot-gpu Docker image (the single-binary driving system).
#
#   ./build.sh                 # tag visionpilot-gpu:latest
#   IMAGE_TAG=mytag ./build.sh # custom tag
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTEXT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)" # the VisionPilot/ directory
IMAGE_NAME="visionpilot-gpu:${IMAGE_TAG:-latest}"

echo "Building ${IMAGE_NAME} (context=${CONTEXT_DIR})..."
docker build -f "${SCRIPT_DIR}/Dockerfile" -t "${IMAGE_NAME}" "${CONTEXT_DIR}"

echo "Build complete: ${IMAGE_NAME}"
