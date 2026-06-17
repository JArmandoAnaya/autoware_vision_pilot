// Smoke test for the perception -> planner -> control bridge: representative inputs must
// yield a finite ControlCommand inside the controllers' physical envelopes.
#include <control/control_bridge.hpp>
#include <control/lateral_control.hpp>
#include <control/longitudinal_control.hpp>

#include <cmath>
#include <cstdio>
#include <string>

namespace {

int g_failures = 0;

void check(const std::string& name, bool ok)
{
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", name.c_str());
    if (!ok) ++g_failures;
}

bool in_envelope(const ControlCommand& c)
{
    const LongitudinalController::Config lc;
    const LateralController::Config tc;
    return std::isfinite(c.steering_angle_rad) && std::isfinite(c.speed_mps) &&
           std::isfinite(c.acceleration_mps2) &&
           std::fabs(c.steering_angle_rad) <= tc.max_steer_rad + 1e-9 &&
           c.speed_mps >= lc.min_speed_mps - 1e-9 && c.speed_mps <= lc.max_speed_mps + 1e-9 &&
           c.acceleration_mps2 >= lc.a_min - 1e-9 && c.acceleration_mps2 <= lc.a_max + 1e-9;
}

// Fresh bridge per scenario (planner/jerk/slew state must not leak between cases).
// cipo_v is the lead's absolute speed; the bridge takes the closing rate (cipo_v - ego_v).
ControlCommand eval(double cte, double epsi, double kappa, double ego_v, bool has_cipo,
                    double cipo_v, double cipo_distance)
{
    ControlBridge bridge;
    return bridge.compute(cte, epsi, kappa, has_cipo, cipo_v - ego_v, cipo_distance, ego_v, 0.1);
}

}  // namespace

int main()
{
    {  // straight free road
        const ControlCommand cmd = eval(0.0, 0.0, 0.0, 10.0, false, 10.0, 9999.0);
        check("straight free road: finite + in envelope", in_envelope(cmd));
        check("straight free road: steering ~0", std::fabs(cmd.steering_angle_rad) < 0.05);
    }
    {  // left curve with a lateral offset, free road
        const ControlCommand cmd = eval(0.3, 0.05, 0.02, 12.0, false, 12.0, 9999.0);
        check("curve: finite + in envelope", in_envelope(cmd));
    }
    {  // approaching a slower lead vehicle (ego 15, lead 10, gap 12 m)
        const ControlCommand cmd = eval(0.0, 0.0, 0.0, 15.0, true, 10.0, 12.0);
        check("lead vehicle: finite + in envelope", in_envelope(cmd));
        check("lead vehicle: decelerates", cmd.acceleration_mps2 <= 0.0);
    }

    std::printf("\n%s (%d failure%s)\n", g_failures ? "FAILED" : "ALL PASS", g_failures,
                g_failures == 1 ? "" : "s");
    return g_failures == 0 ? 0 : 1;
}
