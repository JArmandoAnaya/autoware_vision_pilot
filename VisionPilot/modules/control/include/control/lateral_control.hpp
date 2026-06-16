#ifndef VISIONPILOT_LATERAL_CONTROL_HPP
#define VISIONPILOT_LATERAL_CONTROL_HPP

namespace visionpilot::control {

// Thin adapter over the MPC Planner's lateral output.
//
// The Planner (planning/planning.hpp) already solves the lateral MPC and returns
// a steering sequence; this controller takes the first command and applies a hard
// safety clamp. All lateral logic lives in the Planner.
class LateralController {
public:
    struct Config {
        // Hard clamp on commanded front-wheel angle [rad]. CARLA's largest steer
        // maps from the vehicle's max_steer_angle; ~0.6 rad (≈35°) is generous.
        double max_steering_rad = 0.6;
    };

    LateralController() = default;
    explicit LateralController(const Config& config) : cfg_(config) {}

    // Clamp the Planner's steering_sequence[0] to a commanded front-wheel angle.
    double compute(double planner_steering_rad);

private:
    Config cfg_;
};

}  // namespace visionpilot::control

#endif  // VISIONPILOT_LATERAL_CONTROL_HPP
