# syntax=docker/dockerfile:1
# Self-contained GPU image for the VisionPilot driving binary. The full CUDA 12.8
# + cuDNN 9 + ONNX Runtime stack is baked in from an official nvidia/cuda base so
# ORT can JIT its kernels onto Blackwell sm_120 GPUs; the only runtime host dep is
# the NVIDIA driver (--gpus all). Build context = VisionPilot/; use docker/build.sh
# and docker/run.sh.
ARG CUDA_TAG=12.8.1-cudnn-runtime-ubuntu24.04
FROM nvidia/cuda:${CUDA_TAG}

ARG ROS_DISTRO=jazzy
ARG ORT_VERSION=1.23.2
ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8

# --- ROS2 apt repo + build/runtime dependencies -----------------------------
# Pinning apt versions on a ROS/CUDA base image is impractical.
# hadolint ignore=DL3008
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl gnupg locales \
 && locale-gen en_US.UTF-8 \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu noble main" > /etc/apt/sources.list.d/ros2.list \
 && apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git pkg-config \
        python3 python3-numpy python3-opencv \
        libopencv-dev libyaml-cpp-dev libeigen3-dev libboost-all-dev \
        coinor-libipopt-dev libcppad-dev libblas-dev liblapack-dev gfortran \
        ros-${ROS_DISTRO}-ros-base \
        ros-${ROS_DISTRO}-ackermann-msgs \
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
        ros-${ROS_DISTRO}-fastcdr \
 && rm -rf /var/lib/apt/lists/*

# CppAD includes <coin-or/...>; Debian/Ubuntu ships the IpOpt headers in
# /usr/include/coin, so expose them under the coin-or name the sources expect.
RUN [ -e /usr/include/coin-or ] || ln -s /usr/include/coin /usr/include/coin-or

# --- ONNX Runtime GPU 1.23.2 (CUDA EP, JITs sm_120) -------------------------
# Official C++ SDK tarball (ships lib/ + include/). 1.23.2 is the first release
# that successfully JIT-compiles PTX -> sm_120 at first inference; 1.22 does not.
RUN curl -fsSL -o /tmp/ort.tgz \
        "https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VERSION}/onnxruntime-linux-x64-gpu-${ORT_VERSION}.tgz" \
 && mkdir -p /opt/onnxruntime \
 && tar xzf /tmp/ort.tgz -C /opt/onnxruntime --strip-components=1 \
 && rm /tmp/ort.tgz

# --- Build VisionPilot in the image -----------------------------------------
WORKDIR /opt/visionpilot
COPY . /opt/visionpilot

# NOTE: -DVISIONPILOT_GROUND_HOMOGRAPHY MUST be config/H_carla.yaml for CARLA;
# generate_config bakes build/config/homography_C_matrix.yaml from it.
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && cmake -B build \
        -DENABLE_ROS2_INTERFACE=ON \
        -DONNXRUNTIME_ROOT=/opt/onnxruntime \
        -DPython3_EXECUTABLE=/usr/bin/python3 \
        -DVISIONPILOT_GROUND_HOMOGRAPHY=/opt/visionpilot/config/H_carla.yaml \
 && cmake --build build --target VisionPilot -j"$(nproc)"

# --- Runtime environment ----------------------------------------------------
# ONNX Runtime (+ its CUDA provider) and the CUDA toolkit libs on the loader
# path. cuDNN already lives in /usr/lib/x86_64-linux-gnu from the base image.
ENV LD_LIBRARY_PATH=/opt/onnxruntime/lib:/usr/local/cuda/lib64:${LD_LIBRARY_PATH}

COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["./build/VisionPilot", "--help"]
