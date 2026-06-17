#ifndef CONTROL__CONTROL_COMMAND_HPP_
#define CONTROL__CONTROL_COMMAND_HPP_

// Agnostic physical actuation setpoints. Serialized to CAN (#287) or published as
// Ackermann (#288); pedals/torque are the vehicle ECU/DBW's job, not ours (#269).
struct ControlCommand
{
  double steering_angle_rad = 0.0;  // front-wheel angle, signed (left positive)
  double speed_mps = 0.0;           // target longitudinal speed
  double acceleration_mps2 = 0.0;   // target longitudinal acceleration
};

#endif  // CONTROL__CONTROL_COMMAND_HPP_
