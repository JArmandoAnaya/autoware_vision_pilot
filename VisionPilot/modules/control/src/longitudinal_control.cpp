#include <control/longitudinal_control.hpp>

#include <algorithm>
#include <utility>

std::pair<double, double> LongitudinalController::compute(double accel_cmd, double ego_v, double dt)
{
  // Jerk-limit, then clamp to the acceleration envelope.
  const double jerk_step = config_.jerk_max * dt;
  double accel = std::clamp(accel_cmd, prev_accel_ - jerk_step, prev_accel_ + jerk_step);
  accel = std::clamp(accel, config_.a_min, config_.a_max);
  prev_accel_ = accel;

  // Integrate to a target speed, clamped to the speed envelope.
  const double speed = std::clamp(ego_v + accel * dt, config_.min_speed_mps, config_.max_speed_mps);
  return {speed, accel};
}

void LongitudinalController::reset()
{
  prev_accel_ = 0.0;
}
