#include <control/lateral_control.hpp>

#include <algorithm>

double LateralController::compute(double planner_steer_rad, double dt)
{
  double target = std::clamp(planner_steer_rad, -config_.max_steer_rad, config_.max_steer_rad);

  // Seed on the first call so the slew limit doesn't ramp from an arbitrary zero.
  if (!has_prev_) {
    prev_steer_ = target;
    has_prev_ = true;
    return target;
  }

  // Slew-rate limit, then optional first-order low-pass.
  const double slew_step = config_.slew_max * dt;
  double steer = std::clamp(target, prev_steer_ - slew_step, prev_steer_ + slew_step);
  if (config_.lowpass_tau > 0.0) {
    const double alpha = dt / (config_.lowpass_tau + dt);
    steer = prev_steer_ + alpha * (steer - prev_steer_);
  }

  prev_steer_ = steer;
  return steer;
}

void LateralController::reset()
{
  prev_steer_ = 0.0;
  has_prev_ = false;
}
