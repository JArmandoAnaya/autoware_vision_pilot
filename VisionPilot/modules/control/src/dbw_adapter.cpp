#include <control/dbw_adapter.hpp>

#include <algorithm>
#include <cmath>

bool DbwAdapter::send(const ControlCommand & cmd, double ego_v_mps)
{
  const double steering_deg = std::clamp(
    cmd.steering_angle_rad * 180.0 / M_PI, -config_.max_steer_deg, config_.max_steer_deg);
  const double speed_kmh = std::clamp(cmd.speed_mps * 3.6, 0.0, config_.max_speed_kmh);
  const double wheel_kmh = std::clamp(ego_v_mps * 3.6, 0.0, config_.max_speed_kmh);

  // accel -> accelerator percent + brake. PLACEHOLDER mapping (findings doc 7.2): negative
  // acceleration means brake (no accelerator); positive maps linearly onto 0..100% of a_max.
  const bool brake = cmd.acceleration_mps2 < 0.0;
  const double accelerator_pos =
    brake ? 0.0 : std::clamp(cmd.acceleration_mps2 / config_.a_max * 100.0, 0.0, 100.0);

  return sink_.send(steering_deg, accelerator_pos, brake, speed_kmh, wheel_kmh);
}
