#ifndef CONTROL__CONTROL_BRIDGE_HPP_
#define CONTROL__CONTROL_BRIDGE_HPP_

#include <control/control_command.hpp>
#include <control/controller.hpp>
#include <planning/planning.hpp>

// Integration glue between perception and actuation: owns the Planner (safety_guardian
// MPC) and the agnostic Controller so the whole drive step is a single call from the app.
// It takes plain SI scalars (the perception/fusion fields the caller already has) — it does
// NOT depend on the perception types — and returns the shaped ControlCommand. This lives in
// the control module so the app's main loop carries no control orchestration.
class ControlBridge
{
public:
  ControlBridge() = default;

  // One drive step. cte/epsi/kappa: lateral-fusion road-relative errors (model-view frame).
  // cipo_closing_mps: lead closing rate (negative = approaching); cipo_distance_m: gap.
  // ego_v_mps: measured ego speed; dt: step seconds.
  ControlCommand compute(
    double cte, double epsi, double kappa, bool has_cipo, double cipo_closing_mps,
    double cipo_distance_m, double ego_v_mps, double dt);

private:
  Planner planner_;
  Controller controller_;
};

#endif  // CONTROL__CONTROL_BRIDGE_HPP_
