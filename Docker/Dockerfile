# syntax=docker/dockerfile:1
#
# VisionPilot dev/build container.
#
# The base image is chosen by build.sh:
#   GPU (default) : nvidia/cuda:12.8.1-cudnn-runtime-ubuntu24.04  (run with --gpus all)
#   CPU (--no-cuda): ubuntu:24.04                                 (x86_64 or arm64 / Apple Silicon)
#
# Source is NOT copied in — the repo is bind-mounted at /workspace by run.sh so you
# edit on the host and build inside. Only build/runtime dependencies live in the image.

ARG BASE_IMAGE=nvidia/cuda:12.8.1-cudnn-runtime-ubuntu24.04
FROM ${BASE_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# --- System + build dependencies -------------------------------------------
# hadolint ignore=DL3008,DL3015
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        pkg-config \
        git \
        curl \
        ca-certificates \
        gnupg \
        coinor-libipopt-dev \
        liblapack-dev \
        libblas-dev \
        libcppad-dev \
        libyaml-cpp-dev \
        libeigen3-dev \
        libboost-all-dev \
        python3 \
        python3-dev \
        python3-numpy \
        libjpeg-dev \
        libpng-dev \
        libtiff-dev \
        libavcodec-dev \
        libavformat-dev \
        libswscale-dev \
        libavutil-dev \
        qtbase5-dev \
    && rm -rf /var/lib/apt/lists/*

# CppAD includes <coin-or/...>; guarantee that header dir exists on every Ubuntu version.
RUN [ -e /usr/include/coin-or ] || ln -s coin /usr/include/coin-or || true

# --- OpenCV (built from source) --------------------------------------------
# Ubuntu 24.04 only ships OpenCV 4.6 via apt; build the latest 4.x from source
# into /usr/local (C++ libs + headers + python3 cv2). Qt highgui so cv::imshow
# works under QT_QPA_PLATFORM=offscreen, matching the previous apt build.
ARG OPENCV_VERSION=4.12.0
RUN git clone --depth 1 --branch "${OPENCV_VERSION}" https://github.com/opencv/opencv.git /tmp/opencv \
    && cmake -S /tmp/opencv -B /tmp/opencv/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DOPENCV_GENERATE_PKGCONFIG=ON \
        -DBUILD_LIST=calib3d,features2d,highgui,videoio,imgcodecs,imgproc,video,python3 \
        -DBUILD_opencv_python3=ON \
        -DWITH_QT=ON \
        -DWITH_FFMPEG=ON \
        -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_DOCS=OFF \
        -DBUILD_opencv_apps=OFF \
        -DOPENCV_PYTHON3_INSTALL_PATH=lib/python3.12/dist-packages \
    && cmake --build /tmp/opencv/build -j"$(nproc)" \
    && cmake --build /tmp/opencv/build --target install \
    && ldconfig \
    && rm -rf /tmp/opencv \
    && python3 -c "import cv2; print('built cv2', cv2.__version__)"
# Force the build to find this OpenCV (apt cv_bridge later pulls in a 4.6 as a dep).
ENV OpenCV_DIR=/usr/local/lib/cmake/opencv4

# --- ROS2 Jazzy (ENABLE_ROS2_INTERFACE=ON) ---------------------------------
# hadolint ignore=DL3008
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
        > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-base \
        ros-jazzy-cv-bridge \
        ros-jazzy-image-transport \
        ros-jazzy-rosidl-default-generators \
        python3-colcon-common-extensions \
        python3-setuptools \
    && rm -rf /var/lib/apt/lists/*

# --- ONNX Runtime SDK ------------------------------------------------------
# Tarball name is selected by build.sh: GPU x64 / CPU x64 / CPU aarch64.
ARG ORT_VERSION=1.23.2
ARG ORT_TARBALL=onnxruntime-linux-x64-gpu-1.23.2
RUN curl -fsSL "https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VERSION}/${ORT_TARBALL}.tgz" \
        -o /tmp/ort.tgz \
    && mkdir -p /opt/onnxruntime \
    && tar -xzf /tmp/ort.tgz -C /opt/onnxruntime --strip-components=1 \
    && rm /tmp/ort.tgz \
    && echo /opt/onnxruntime/lib > /etc/ld.so.conf.d/onnxruntime.conf \
    && ldconfig
ENV ONNXRUNTIME_ROOT=/opt/onnxruntime

# --- Helper scripts --------------------------------------------------------
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
COPY build_and_test.sh /usr/local/bin/build_and_test.sh
RUN chmod +x /usr/local/bin/entrypoint.sh /usr/local/bin/build_and_test.sh

WORKDIR /workspace
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
