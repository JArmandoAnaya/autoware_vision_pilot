#include "control_bridge.hpp"

#include <utility>
#include <vector>

ControlCommand compute_control_command(
    Planner& planner, LongitudinalController& lon, LateralController& lat,
    double cte, double epsi, double kappa, double ego_v,
    bool has_cipo, double cipo_v, double cipo_distance, double dt)
{
    auto [accel, steer_seq] =
        planner.compute_plan(cte, epsi, kappa, ego_v, has_cipo, cipo_v, cipo_distance);
    const auto [speed_set, accel_set] = lon.compute(accel, ego_v, dt);
    const double steer = steer_seq.empty() ? 0.0 : lat.compute(steer_seq.front(), dt);
    return {steer, speed_set, accel_set};
}
