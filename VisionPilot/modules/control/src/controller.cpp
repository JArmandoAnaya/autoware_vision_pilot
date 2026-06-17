#include <control/controller.hpp>

ControlCommand Controller::compute(
  double steering_angle_rad, double acceleration_mps2, double ego_v_mps, double dt)
{
  const auto [speed_set, accel_set] = lon_.compute(acceleration_mps2, ego_v_mps, dt);
  const double steer = lat_.compute(steering_angle_rad, dt);
  return {steer, speed_set, accel_set};
}

void Controller::reset()
{
  lon_.reset();
  lat_.reset();
}
