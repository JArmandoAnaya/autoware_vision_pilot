#include <control/control_bridge.hpp>
#include <logging/logger.hpp>

ControlCommand ControlBridge::compute(
  double cte, double epsi, double kappa, bool has_cipo, double cipo_closing_mps,
  double cipo_distance_m, double ego_v_mps, double dt)
{
  // Lead absolute speed = ego + closing rate; far sentinel distance when no lead.
  const double cipo_v = has_cipo ? ego_v_mps + cipo_closing_mps : ego_v_mps;
  const double cipo_distance = has_cipo ? cipo_distance_m : 9999.0;

  // Planner (safety_guardian MPC) owns the control law; the Controller shapes its
  // steering-angle + acceleration intent for actuation.
  auto [accel, steer_seq] =
    planner_.compute_plan(cte, epsi, kappa, ego_v_mps, has_cipo, cipo_v, cipo_distance);
  const double planner_steer = steer_seq.empty() ? 0.0 : steer_seq.front();
  const ControlCommand cmd = controller_.compute(planner_steer, accel, ego_v_mps, dt);

  VP_INFO(
    "[Control] steer=%.4f rad  speed=%.2f m/s  accel=%.2f m/s2", cmd.steering_angle_rad,
    cmd.speed_mps, cmd.acceleration_mps2);
  return cmd;
}
