# Control module

Middleware- and vehicle-agnostic layer that turns the Planner's physical intent
(acceleration in m/s², steering angle in rad) into one `ControlCommand`
`{steering_angle_rad, speed_mps, acceleration_mps2}`.

## Contract (upstream issues)

- **#287** — steering angle + vehicle speed are the canonical output, serialized to CAN.
- **#288** — CAN is the primary path; ROS2/Ackermann is an adapter.
- **#269** — VisionPilot emits physical setpoints; the vehicle ECU/DBW owns pedals/torque.

## Components

- **`ControlCommand`** (`control_command.hpp`) — the agnostic output struct.
- **`LongitudinalController`** — integrates the planner's acceleration into a target
  speed, jerk-limited and clamped to a physical envelope. No throttle/brake and no PI
  loop: the ECU/sim closes the actuator loop (#269).
- **`LateralController`** — shapes the MPC steering angle (clamp → slew → low-pass). A
  shaping/actuation layer, **not** a second path-tracker: the MPC owns the steering law,
  and `max_steer_rad` is the vehicle's physical limit, not a control gain.

## Tests

`tests/control/test_control` — deterministic unit + closed-loop kinematic tests
(no model weights, no simulator). Build the `test_control` target and run the binary.
