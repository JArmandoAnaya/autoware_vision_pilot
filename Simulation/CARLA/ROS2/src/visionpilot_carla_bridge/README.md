# visionpilot_carla_bridge

Bridges VisionPilot's middleware-agnostic **Float64** actuation contract to CARLA 0.10
Shipping's native `--ros2` interface, so the camera-only VisionPilot loop can drive the
CARLA ego.

## Contract

VisionPilot (built with `-DENABLE_ROS2_INTERFACE=ON`) publishes/subscribes:

| Topic | Type | Meaning |
|-------|------|---------|
| `/vehicle/steering_cmd` | `std_msgs/Float64` | tyre angle (rad), out |
| `/vehicle/throttle_cmd` | `std_msgs/Float64` | longitudinal accel (m/s²), out |
| `/vehicle/speed` | `std_msgs/Float64` | ego ground speed (m/s), in |

CARLA 0.10 Shipping has **no `ros2_ackermann_control`**, so the ego only accepts
`carla_msgs/CarlaEgoVehicleControl` on `/carla/hero/vehicle_control_cmd`, and publishes the
camera natively at `/carla/hero/main_cam/image`.

## Nodes

- **`carla_control_relay`** — caches the latest `/vehicle/steering_cmd` + `/vehicle/throttle_cmd`
  and publishes `carla_msgs/CarlaEgoVehicleControl` at a fixed rate. Mapping (pure, unit-tested in
  `control_mapping.py`): `steer = clamp(steer_rad / max_steer, ±1)`; **direct accel→throttle**
  `accel>0 → throttle=clamp(k_t·accel)`, `accel<0 → brake=clamp(k_b·|accel|)`, else coast.
  Params: `steering_topic`, `throttle_topic`, `control_topic`, `control_rate_hz`, `max_steer`,
  `throttle_gain`, `brake_gain`, `accel_deadband`, `throttle_max`.
- **`ego_speed_republisher`** — `nav_msgs/Odometry` → `/vehicle/speed` Float64
  (`speed = hypot(vx, vy)`). Use when an Odometry source is already on the graph.
  Params: `odom_topic`, `speed_topic`.

Ego speed without an Odometry source: run `../../ego_speed_publisher.py` (CARLA PythonAPI),
which reads the hero actor velocity and publishes `/vehicle/speed` directly.

## Build

Self-contained colcon package — builds identically on a host with ROS 2 Jazzy or in the
VisionPilot dev container. From the colcon workspace root (`Simulation/CARLA/ROS2`):

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select carla_msgs visionpilot_carla_bridge
source install/setup.bash
```

In the dev container (no host ROS 2 needed):

```bash
./Docker/run.sh -- bash -lc 'source /opt/ros/jazzy/setup.bash && \
  cd /workspace/Simulation/CARLA/ROS2 && \
  colcon build --packages-select carla_msgs visionpilot_carla_bridge'
```

> The other packages under `src/` are the legacy shared-memory CARLA pipeline; build only the
> two above (`--packages-select`) — a bare `colcon build` will try them too.

## Test

```bash
cd src/visionpilot_carla_bridge && PYTHONPATH=$PWD python3 -m pytest test/ -v
```

## Run

```bash
ros2 launch visionpilot_carla_bridge carla_bridge.launch.py
# or individually:
ros2 run visionpilot_carla_bridge carla_control_relay
ros2 run visionpilot_carla_bridge ego_speed_republisher
```

See `Simulation/CARLA/ROS2/README.md` for the full CARLA bring-up.
