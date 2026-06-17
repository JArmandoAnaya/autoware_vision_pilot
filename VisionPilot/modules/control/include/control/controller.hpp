#ifndef CONTROL__CONTROLLER_HPP_
#define CONTROL__CONTROLLER_HPP_

#include <control/control_command.hpp>
#include <control/lateral_control.hpp>
#include <control/longitudinal_control.hpp>

// Routes the planner's physical intent (front-wheel steering angle + longitudinal
// acceleration) through the longitudinal and lateral shaping controllers into a single
// ControlCommand. The planner (safety_guardian MPC) owns the control laws; this facade is
// the one entry point the app calls per frame. It is given steering angle and acceleration
// and does NOT run the planner itself.
class Controller
{
public:
  Controller() = default;
  Controller(const LongitudinalController::Config & lon, const LateralController::Config & lat)
  : lon_(lon), lat_(lat)
  {
  }

  // steering_angle_rad, acceleration_mps2 = Planner::compute_plan() outputs;
  // ego_v_mps = measured ego speed; dt = step seconds.
  ControlCommand compute(
    double steering_angle_rad, double acceleration_mps2, double ego_v_mps, double dt);

  void reset();  // clear longitudinal jerk + lateral slew/low-pass state

  LongitudinalController & longitudinal() { return lon_; }
  LateralController & lateral() { return lat_; }

private:
  LongitudinalController lon_;
  LateralController lat_;
};

#endif  // CONTROL__CONTROLLER_HPP_
