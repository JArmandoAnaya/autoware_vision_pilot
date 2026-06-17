#ifndef CONTROL__CONTROL_COMMAND_HPP_
#define CONTROL__CONTROL_COMMAND_HPP_

// Agnostic physical actuation setpoints in SI units. A transport adapter turns this into
// the middleware/vehicle output (ROS2 Ackermann for Standalone). Pedals/torque and any CAN
// serialization are the vehicle ECU / sensing layer's job, not the control module's.
struct ControlCommand
{
  double steering_angle_rad = 0.0;  // front-wheel angle, signed (left positive)
  double speed_mps = 0.0;           // target longitudinal speed
  double acceleration_mps2 = 0.0;   // target longitudinal acceleration
};

#endif  // CONTROL__CONTROL_COMMAND_HPP_
