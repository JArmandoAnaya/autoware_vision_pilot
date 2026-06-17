#ifndef CONTROL__LATERAL_CONTROL_HPP_
#define CONTROL__LATERAL_CONTROL_HPP_

// Shapes the MPC's steering angle for actuation: clamp to the vehicle limit, slew-rate
// limit, optional first-order low-pass. It is a shaping layer, NOT a second path-tracker
// (the MPC owns the steering law); max_steer_rad is the physical limit, not a gain.
class LateralController
{
public:
  struct Config
  {
    double max_steer_rad = 0.6109;  // vehicle physical limit (~35 deg)
    double slew_max = 5.0;          // rad/s
    double lowpass_tau = 0.05;      // s; 0 disables
  };

  LateralController() = default;
  explicit LateralController(const Config & config) : config_(config) {}

  // planner_steer from Planner::compute_plan().second.front(); returns shaped angle (rad).
  double compute(double planner_steer_rad, double dt);

  void reset();  // clear slew/low-pass state

  const Config & config() const { return config_; }

private:
  Config config_;
  double prev_steer_ = 0.0;
  bool has_prev_ = false;
};

#endif  // CONTROL__LATERAL_CONTROL_HPP_
