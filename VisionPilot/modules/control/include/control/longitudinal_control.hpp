#ifndef CONTROL__LONGITUDINAL_CONTROL_HPP_
#define CONTROL__LONGITUDINAL_CONTROL_HPP_

#include <utility>

// Integrates the planner's acceleration intent into a target speed, jerk-limited and
// clamped to a physical envelope. Emits a setpoint, not pedals: the ECU/sim closes the
// actuator loop (#269), so there is no throttle/brake and no speed-tracking PI loop.
class LongitudinalController
{
public:
  struct Config
  {
    double max_speed_mps = 60.0 / 3.6;  // envelope ceiling
    double min_speed_mps = 0.0;         // floor (no reverse)
    double a_max = 1.5;                 // m/s^2 (matches the planner's comfort accel)
    double a_min = -3.0;                // m/s^2 (comfort decel)
    double jerk_max = 2.5;              // m/s^3 — slew rate on the commanded acceleration
  };

  LongitudinalController() = default;
  explicit LongitudinalController(const Config & config) : config_(config) {}

  // accel_cmd from Planner::compute_plan().first; returns {speed_mps, accel_mps2}.
  std::pair<double, double> compute(double accel_cmd, double ego_v, double dt);

  void reset();  // clear jerk-limiting state

  const Config & config() const { return config_; }

private:
  Config config_;
  double prev_accel_ = 0.0;  // last commanded acceleration, for jerk limiting
};

#endif  // CONTROL__LONGITUDINAL_CONTROL_HPP_
