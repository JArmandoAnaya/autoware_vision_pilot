# Control module

Middleware- and vehicle-agnostic layer that turns the Planner's physical intent
(acceleration in m/s², steering angle in rad) into one `ControlCommand`
`{steering_angle_rad, speed_mps, acceleration_mps2}`.

## Contract

The control module emits one agnostic `ControlCommand` of physical setpoints. It owns **no**
transport: turning the command into a vehicle/middleware output is a separate layer's job. CAN
read/write is the **sensing** module's concern, not control's, and Standalone does not use CAN.

## Components

- **`ControlCommand`** (`control_command.hpp`) — the agnostic output struct.
- **`LongitudinalController`** — integrates the planner's acceleration into a target
  speed, jerk-limited and clamped to a physical envelope. No throttle/brake and no PI
  loop: the actuator loop is closed downstream (ECU/sim).
- **`LateralController`** — shapes the MPC steering angle (clamp → slew → low-pass). A
  shaping/actuation layer, **not** a second path-tracker: the MPC owns the steering law,
  and `max_steer_rad` is the vehicle's physical limit, not a control gain.
- **`Controller`** (`controller.hpp`) — facade over the two controllers: given the planner's
  steering angle + acceleration, routes them to the lateral/longitudinal controllers and
  returns the shaped `ControlCommand`.
- **`ControlBridge`** (`control_bridge.{hpp,cpp}`) — integration glue owning the Planner +
  `Controller`, so the app computes a full drive step (perception error → `ControlCommand`)
  in one call.

## Actuation output (where the `ControlCommand` goes)

For Standalone the command is published as **ROS2 Ackermann** by
`modules/middleware_interfaces/ros2_interface/control_cmd_publisher` (`ENABLE_ROS2_INTERFACE`).
Ego speed for the longitudinal loop comes back via `.../vehicle_state_subscriber` (odometry).
Any CAN/DBW transport, if added, lives in the sensing layer and consumes the same agnostic
`ControlCommand` — the control module stays bus-agnostic.

## Config (in `vision_pilot.conf`)

`control.enabled` (off by default → pure perception + display), `control.ego_speed_mps` (fallback
until live odometry), `control.dt_s` (control period), plus the ROS2 keys documented in the
publisher/subscriber module READMEs.

## Tests (deterministic, no weights, no simulator)

Build the target and run the binary; each prints `PASS`/`FAIL` and exits non-zero on failure.

- `test_control` — unit + closed-loop kinematic tests for the two controllers.
- `test_closed_loop` — **closed-loop SIL harness**: the real stack
  (`compute_plan`→longitudinal→lateral) driven around an independent kinematic plant on a
  straight + an R=50 m curve; asserts a sign-convention check, bounded CTE, and speed tracking.
