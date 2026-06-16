#include <control/lateral_control.hpp>

#include <algorithm>

namespace visionpilot::control {

double LateralController::compute(double planner_steering_rad)
{
    // The Planner's MPC output is the commanded steering; apply only a hard
    // safety clamp on the front-wheel angle.
    return std::clamp(planner_steering_rad, -cfg_.max_steering_rad, cfg_.max_steering_rad);
}

}  // namespace visionpilot::control
