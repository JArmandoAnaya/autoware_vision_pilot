#include <control/longitudinal_control.hpp>

#include <algorithm>

namespace visionpilot::control {

double LongitudinalController::compute(double accel_mps2, double ego_v_mps)
{
    const double target = ego_v_mps + accel_mps2 * cfg_.lookahead_s;
    return std::clamp(target, cfg_.min_cruise_mps, cfg_.max_speed_mps);
}

}  // namespace visionpilot::control
