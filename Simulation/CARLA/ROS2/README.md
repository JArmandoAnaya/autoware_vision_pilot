# VisionPilot ⇄ CARLA 0.10 (ROS 2)

Drive the CARLA 0.10 Shipping ego **camera-only** with the single-binary VisionPilot
closed loop (perception → planning → control → actuation), over CARLA's native `--ros2`.
**VisionPilot itself is unmodified** — everything CARLA-specific lives here.

```
CARLA 0.10 --ros2 (windowed)
  /carla/hero/main_cam/image  ─ sensor_msgs/Image ─►  VisionPilot (camera_subscriber)
  ego velocity ─► ego_speed_publisher ─ /vehicle/speed (Float64) ─►  VisionPilot
VisionPilot  (ENABLE_ROS2_INTERFACE=ON, source.mode=ros2)
  /vehicle/steering_cmd (Float64 rad) ┐
  /vehicle/throttle_cmd (Float64 m/s²)┴─►  carla_control_relay ─► /carla/hero/vehicle_control_cmd
                                                              (carla_msgs/CarlaEgoVehicleControl)
```

## Layout

| Path | Role |
|------|------|
| `src/visionpilot_carla_bridge/` | `carla_control_relay` (Float64 cmds → `CarlaEgoVehicleControl`) + `ego_speed_republisher`; colcon, host- or container-buildable |
| `src/carla_msgs/` | vendored `CarlaEgoVehicleControl.msg` |
| `ros_carla_config.py` | spawn `hero` ego + `main_cam` (CARLA PythonAPI, async mode); `SPAWN_INDEX` env |
| `ego_speed_publisher.py` | CARLA-PythonAPI ego speed → `/vehicle/speed` (CARLA 0.10 emits no usable odometry) |
| `config/VisionPilot_carla10.json` | ego + sensors (`hero`, `main_cam` 1280×720 **fov 60** z 1.58) |
| `config/H_carla.yaml` | CARLA main_cam ground homography (camera → world) |
| `config/homography_C_matrix.yaml` | preprocess warp matrix C (camera → model view), derived from `H_carla.yaml` |
| `config/visionpilot.carla.conf`, `config/visionpilot_ros2.carla.conf` | VisionPilot run config overlay for CARLA (ros2 source + `/vehicle/*` topics) |
| `stage_carla_config.sh` | install the two `.conf` + the C matrix into `VisionPilot/config/` for a run |
| `gen_carla_homography.py` | regenerate `H_carla.yaml` if the camera geometry changes |
| `build_bridge.sh` | colcon-build the bridge (`carla_msgs` + `visionpilot_carla_bridge`) |

> **Camera FOV is 60°, not the model's ~42° view.** The homography warp reprojects the camera
> into the model view, sampling slightly *beyond* the model's frame at the edges; the camera must
> be **wider** than the model view so that reprojection stays inside the captured image (a narrower
> fov makes the warp read out-of-frame → a mirror/ghost artifact). The warp crops to the model view.

## Build

```bash
# 1) VisionPilot with the ROS2 interface (in the dev container — see /Docker):
#    cmake -B build_docker_ros2 -DENABLE_ROS2_INTERFACE=ON ... && cmake --build build_docker_ros2 --target VisionPilot
# 2) The bridge (host ROS 2 Jazzy, or the dev container):
./build_bridge.sh
source install/setup.bash
```

## Bring-up (fresh full restart every run)

```bash
# 0) stage the CARLA config into VisionPilot/config/ (does NOT modify tracked defaults in git)
./stage_carla_config.sh

# 1) CARLA — windowed, native ROS 2
"$CARLA_ROOT/CarlaUnreal.sh" -nosound --ros2

# 2) spawn ego + camera (CARLA PythonAPI; SPAWN_INDEX picks the spawn point)
SPAWN_INDEX=100 python3 ros_carla_config.py -f config/VisionPilot_carla10.json

# 3) ego speed -> /vehicle/speed
python3 ego_speed_publisher.py

# 4) bridge — VisionPilot Float64 actuation -> CARLA control
ros2 launch visionpilot_carla_bridge carla_bridge.launch.py

# 5) VisionPilot (CWD = VisionPilot/, ROS2 build), DDS over UDP
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ./build_docker_ros2/VisionPilot
```

All components share `--net=host` + `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`. Between experiments do a
full teardown (kill CARLA, the spawn helper, the speed publisher, the bridge, the VisionPilot
process) — stale DDS topics from a previous run interfere with the next one.

`carla_speed_monitor.py` is an optional CARLA-PythonAPI observability tool (ground-truth ego speed +
a collision sensor). `docs/` holds the workflow diagram.
