#ifndef VISIONPILOT_LONGITUDINAL_CONTROL_HPP
#define VISIONPILOT_LONGITUDINAL_CONTROL_HPP

namespace visionpilot::control {

// Thin adapter over the MPC Planner's longitudinal output.
//
// Ackermann actuation tracks a target *speed*, but the Planner emits an
// *acceleration* (RSS / following-distance already applied inside it). This
// controller integrates that acceleration into a target speed over a short
// lookahead and clamps it to the operating envelope. It adds no new longitudinal
// logic of its own.
class LongitudinalController {
public:
    struct Config {
        double min_cruise_mps = 0.0;          // floor for the commanded speed
        double max_speed_mps  = 60.0 / 3.6;   // ceiling (default 60 km/h)
        // Speed-command lookahead: the Ackermann sink tracks a *target speed*, so
        // we command the speed the planner wants ~lookahead_s ahead
        // (v = ego_v + accel*lookahead_s). One MPC step (0.05 s) is far too short —
        // from standstill it yields a near-zero target the sink reads as "stop".
        double lookahead_s    = 1.0;
    };

    LongitudinalController() = default;
    explicit LongitudinalController(const Config& config) : cfg_(config) {}

    // Integrate planner acceleration into an Ackermann target speed [m/s]:
    //   v = clamp(ego_v + accel*lookahead_s, min_cruise_mps, max_speed_mps)
    double compute(double accel_mps2, double ego_v_mps);

private:
    Config cfg_;
};

}  // namespace visionpilot::control

#endif  // VISIONPILOT_LONGITUDINAL_CONTROL_HPP
