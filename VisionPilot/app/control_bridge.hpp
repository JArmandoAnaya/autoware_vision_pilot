#pragma once

#include <control/control_command.hpp>
#include <control/lateral_control.hpp>
#include <control/longitudinal_control.hpp>
#include <planning/planning.hpp>

// Application glue: run the planner's intent through the controllers into a shaped
// ControlCommand. The planner and controllers are stateful, so they are passed by
// reference. cipo_v is the lead's absolute speed (free road: pass ego_v, distance 9999).
ControlCommand compute_control_command(
    Planner& planner, LongitudinalController& lon, LateralController& lat,
    double cte, double epsi, double kappa, double ego_v,
    bool has_cipo, double cipo_v, double cipo_distance, double dt);
