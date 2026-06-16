# VisionPilot on CARLA UE5 (camera-only drive)

Run the VisionPilot single-binary driving system on a CARLA UE5 0.10 server, off a
**single front camera**: camera in → perception → fusion → MPC planning → control →
Ackermann out. Control lives inside the C++ binary; there is no Python control relay.

Everything runs in Docker. The **only** host dependencies are the NVIDIA driver, Docker
(with the NVIDIA runtime), and your CARLA UE5 server. CUDA, cuDNN, ONNX Runtime, ROS2 and
the CARLA Python client are all baked into the images.

```text
                 CARLA UE5 0.10  (--ros2, FASTDDS_BUILTIN_TRANSPORTS=UDPv4)
                   │  camera /carla/hero/main_cam/image
                   ▼
        ┌─────────────────────┐   ackermann    ┌──────────────────────┐  vehicle_control_cmd
        │   visionpilot-gpu   │ ─────────────▶ │  carla-control-bridge │ ───────────────────▶ CARLA ego
        │  (perception→MPC)   │                └──────────────────────┘
        └─────────────────────┘ ◀─ odometry ── ┌──────────────────────┐
                                                │   carla-odom-bridge  │ ◀── CARLA RPC odometry
                                                └──────────────────────┘
            carla-spawn  ── spawns ego + main_cam, holds it stationary until VisionPilot is ready
```

> **Why two bridges?** CARLA UE5 0.10 _Shipping_ builds expose no `ros2_ackermann_control`
> and no native ROS2 odometry. The ego only accepts `carla_msgs/CarlaEgoVehicleControl` on
> `/carla/hero/vehicle_control_cmd`. VisionPilot stays middleware-agnostic (it publishes
> generic `ackermann_msgs` + reads generic `nav_msgs/Odometry`); the two small bridge
> containers do the CARLA-specific translation. Everything CARLA-specific is config, not code.

---

## Prerequisites

1. **NVIDIA driver + Docker with GPU access** (`docker run --rm --gpus all nvidia/cuda:12.8.1-base-ubuntu24.04 nvidia-smi` works).
2. **CARLA UE5 0.10** installed on the host (the `CarlaUnreal.sh` server).
3. **CARLA Python 3.10 wheel** — needed to build the odom bridge. Download from the
   [CARLA releases page](https://github.com/carla-simulator/carla/releases)
   (`carla-0.10.0-cp310-cp310-linux_x86_64.whl`, under _Additional Maps & Assets → PythonAPI_).
4. **Model weights** (not in git) — the **FP32 ONNX** weights for AutoDrive, AutoSteer and
   AutoSpeed, from the Google Drive links in `Models/model_library/{AutoDrive,AutoSteer,AutoSpeed}/README.md`
   (the "Download ONNX FP32 Weights \*.onnx" line in each). See the download step below.

---

## Setup (once)

```bash
cd Simulation/CARLA/ROS2/docker

# 1. CARLA Python wheel (for the odom bridge)
export CARLA_WHL=/path/to/carla-0.10.0-cp310-cp310-linux_x86_64.whl

# 2. Download the three FP32 ONNX weights into VisionPilot/models/ with the exact
#    names the config expects. The Drive file IDs below are the "ONNX FP32" links
#    from Models/model_library/{AutoDrive,AutoSteer,AutoSpeed}/README.md.
pip install gdown   # one-time, if you don't have it
MODELS=../../../../VisionPilot/models
gdown 1GKqhrNP5xtLBRqrcqL9k1IKnwicSL-N8 -O "$MODELS/autodrive.onnx"      # AutoDrive  FP32
gdown 1u89PujOd89M-l6t_Cvub3BaQsDFKofuY -O "$MODELS/autosteer_2.0.onnx"  # AutoSteer 2.0 FP32
gdown 1bKYsnKbHD8DvQB2w3x6yu0Htup4KL2l5 -O "$MODELS/autospeed_2.onnx"    # AutoSpeed 2 FP32

# 3. Build all three images
./build_all.sh
```

The default drive config is `VisionPilot/config/vision_pilot_carla.conf.example` (already set
up for CARLA: GPU inference, the topic overlay, weights at `/weights/*.onnx`). To customise it,
copy it and point `VP_CONFIG` at the copy.

---

## Drive (each run)

```bash
# Terminal 1 — your CARLA server (UE5 0.10), native ROS2 over UDP:
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ./CarlaUnreal.sh --ros2

# Terminal 2 — everything else, one command:
Simulation/CARLA/ROS2/docker/drive.sh
```

`drive.sh` does a clean teardown, then starts the odom bridge, the control bridge, the spawn
helper (it spawns the ego at the verified curved-lane spawn on Town10 and holds it stationary
until VisionPilot's first command), and finally the `visionpilot-gpu` binary in the foreground.
**Press Ctrl-C to stop everything** — the trap tears the whole stack down.

Verify the actuation is flowing (from any ROS2 shell, or `docker exec` into a bridge):

```bash
ros2 topic echo /carla/hero/vehicle_control_cmd     # throttle / steer / brake to the ego
ros2 topic echo /carla/hero/ackermann_control_cmd   # VisionPilot's raw ackermann output
```

### Fresh restart between runs (important)

Stale ROS2/DDS publishers from a previous run interpolate with a new one (duplicate publishers
→ orphan ego / "collision at spawn position"). `drive.sh` tears its own containers down on start
and exit. **Also restart the CARLA server itself between experiments** — don't reuse a CARLA that
has been driven already. To tear down the containers without driving: `./down.sh`.

---

## Topics

| Direction       | Topic                               | Type                                   | Who                          | Config key                    |
| --------------- | ----------------------------------- | -------------------------------------- | ---------------------------- | ----------------------------- |
| in (camera)     | `/carla/hero/main_cam/image`        | `sensor_msgs/Image`                    | CARLA → VisionPilot          | `source.ros2_topic`           |
| in (ego state)  | `/carla/hero/odometry`              | `nav_msgs/Odometry`                    | odom bridge → VisionPilot    | `control.vehicle_state_topic` |
| out (ackermann) | `/carla/hero/ackermann_control_cmd` | `ackermann_msgs/AckermannDriveStamped` | VisionPilot → control bridge | `control.topic`               |
| out (actuation) | `/carla/hero/vehicle_control_cmd`   | `carla_msgs/CarlaEgoVehicleControl`    | control bridge → CARLA       | — (bridge `CTRL_TOPIC`)       |

`steering_angle` is the real front-wheel angle (rad); the control bridge normalises it to
CARLA's `[-1, 1]` steer (`MAX_STEER_RAD`) and converts the target `speed` (m/s) to
throttle/brake with a P-controller on the odometry speed.

---

## Manual / advanced

The orchestrator just sequences the per-component scripts; you can run them individually:

| Component              | Build                                             | Run                            |
| ---------------------- | ------------------------------------------------- | ------------------------------ |
| VisionPilot GPU binary | `VisionPilot/docker/build.sh`                     | `VisionPilot/docker/run.sh`    |
| Odom bridge            | `docker/odom_bridge/build.sh` (needs `CARLA_WHL`) | `docker/odom_bridge/run.sh`    |
| Control bridge         | `docker/control_bridge/build.sh`                  | `docker/control_bridge/run.sh` |

`VisionPilot/docker/run.sh` env overrides: `VP_CONFIG` (host conf path), `VP_WEIGHTS` (host
weights dir, mounted ro at `/weights`), `VP_MODEL_CACHE` (shared CUDA/ONNX JIT cache — keeps
repeat starts fast), `VP_IPC_DIR` (readiness handshake dir).

**Building the binary natively** (no Docker) is also possible — in a ROS2 **Jazzy** environment
with IPOPT + CppAD, OpenCV and an ONNX Runtime SDK:

```bash
apt-get install -y coinor-libipopt-dev libcppad-dev
ln -s /usr/include/coin /usr/include/coin-or          # CppAD expects <coin-or/...>
source /opt/ros/jazzy/setup.bash
cd VisionPilot
cmake -B build -DENABLE_ROS2_INTERFACE=ON \
  -DONNXRUNTIME_ROOT=$ONNXRUNTIME_ROOT \
  -DVISIONPILOT_GROUND_HOMOGRAPHY=$PWD/config/H_carla.yaml   # MUST be H_carla.yaml for CARLA
cmake --build build --target VisionPilot -j$(nproc)
```

> CARLA-specific homography is `config/H_carla.yaml`; the build's `generate_config` step bakes
> `build/config/homography_C_matrix.yaml` from it. Using `config/H.yaml` (a generic example)
> gives the wrong ground projection for CARLA.

---

## Fast-DDS transport (why `UDPv4` everywhere)

CARLA's native ROS2 uses Fast-DDS, which defaults to **shared-memory** transport between
same-host peers. Shared memory does **not** cross a host↔container boundary — even with
`--net=host --ipc=host`. Symptom: `ros2 topic list` shows the topics, but no camera frames
arrive and control commands are silently ignored (the car never moves), because discovery uses
small UDP packets while the bulk/control data goes over the unreachable shared-memory channel.
Forcing `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` on **both** sides (the CARLA server and every
container — the scripts already set it) fixes this. Use `LARGE_DATA` instead if large images
show fragmentation. Native same-host, no containers: shared memory works and the flag is unneeded.

---

## CARLA 0.9 server

`config/VisionPilot_carla9.json` targets CARLA 0.9.16, whose server binary is `CarlaUE4.sh`
(not `CarlaUnreal.sh`). The drive path above is validated on UE5 0.10.

## Scenario runner

```bash
git clone https://github.com/carla-simulator/scenario_runner.git
cd scenario_runner && python3 -m venv .venv && pip3 install -r requirements.txt
export CARLA_ROOT=<CARLA ROOT> PYTHONPATH=$CARLA_ROOT/PythonAPI/carla
python3 ./scenario_runner.py --openscenario ./srunner/examples/LaneChangeSimple.xosc
```

## Optional eval / visualization packages (not in the drive path)

`camera_publisher`, `camera_spectator`, `odom_publisher`, `road_shape_publisher`,
`waypoints_publisher` — ROS2 packages useful for debugging/visualization, built with `colcon`.
Not required to drive.
