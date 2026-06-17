// Deterministic unit + closed-loop tests for the control module. No weights, no sim.
#include <control/control_command.hpp>
#include <control/lateral_control.hpp>
#include <control/longitudinal_control.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace
{

int g_failures = 0;

void check(const std::string & name, bool ok)
{
  std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", name.c_str());
  if (!ok) ++g_failures;
}

bool approx(double a, double b, double tol)
{
  return std::fabs(a - b) <= tol;
}

// ---- Longitudinal unit tests ------------------------------------------------

void test_longitudinal_unit()
{
  {  // zero command holds speed
    LongitudinalController c;
    auto [v, a] = c.compute(0.0, 10.0, 0.1);
    check("long: zero accel holds speed", approx(v, 10.0, 1e-9) && approx(a, 0.0, 1e-9));
  }
  {  // command within the jerk budget passes through
    LongitudinalController c;
    auto [v, a] = c.compute(0.1, 10.0, 0.1);  // jerk budget = 2.5*0.1 = 0.25
    check("long: small accel passes through", approx(a, 0.1, 1e-9) && approx(v, 10.01, 1e-9));
  }
  {  // huge accel is jerk-limited on the first step, then saturates at a_max
    LongitudinalController c;
    auto [v0, a0] = c.compute(100.0, 0.0, 0.1);
    bool jerk_ok = approx(a0, 0.25, 1e-9);
    double v = v0, a = a0;
    for (int i = 0; i < 50; ++i) {
      auto r = c.compute(100.0, v, 0.1);
      v = r.first;
      a = r.second;
    }
    check("long: accel jerk-limited then clamped to a_max", jerk_ok && approx(a, 1.5, 1e-9));
  }
  {  // huge decel saturates at a_min and speed is floored at 0
    LongitudinalController c;
    double v = 5.0;
    double a = 0.0;
    bool floored = true;
    for (int i = 0; i < 100; ++i) {
      auto r = c.compute(-100.0, v, 0.1);
      v = r.first;
      a = r.second;
      if (v < 0.0) floored = false;
    }
    check(
      "long: decel clamped to a_min, speed floored",
      approx(a, -3.0, 1e-9) && floored && approx(v, 0.0, 1e-9));
  }
  {  // speed never exceeds the envelope ceiling
    LongitudinalController::Config cfg;
    cfg.max_speed_mps = 12.0;
    LongitudinalController c(cfg);
    double v = 10.0;
    bool capped = true;
    for (int i = 0; i < 200; ++i) {
      auto r = c.compute(5.0, v, 0.1);
      v = r.first;
      if (v > 12.0 + 1e-9) capped = false;
    }
    check("long: speed capped at max_speed", capped && approx(v, 12.0, 1e-6));
  }
}

// ---- Lateral unit tests -----------------------------------------------------

void test_lateral_unit()
{
  {  // steady input converges to that value
    LateralController c;
    double s = 0.0;
    for (int i = 0; i < 200; ++i) s = c.compute(0.1, 0.02);
    check("lat: steady input converges", approx(s, 0.1, 1e-3));
  }
  {  // a step is slew-rate limited relative to the previous output
    LateralController c;
    c.compute(0.0, 0.02);  // seed previous at 0
    double s = c.compute(0.5, 0.02);
    check("lat: step is slew limited", std::fabs(s - 0.0) <= 5.0 * 0.02 + 1e-9);
  }
  {  // over-limit command is clamped to the physical limit
    LateralController c;
    double s = 0.0;
    for (int i = 0; i < 200; ++i) s = c.compute(2.0, 0.02);
    check("lat: clamped to max_steer", approx(s, 0.6109, 1e-4));
  }
  {  // sign is preserved
    LateralController c;
    double s = 0.0;
    for (int i = 0; i < 200; ++i) s = c.compute(-0.2, 0.02);
    check("lat: sign preserved", s < 0.0 && approx(s, -0.2, 1e-3));
  }
}

// ---- Closed-loop tests (proportional laws below are test scaffolding) --------

void test_longitudinal_closed_loop()
{
  // Point mass; a proportional law stands in for the planner asking for v_target.
  LongitudinalController c;
  const double v_target = 15.0, dt = 0.1, kp = 0.5;
  double v = 0.0, overshoot = 0.0;
  for (int i = 0; i < 1000; ++i) {
    auto r = c.compute(kp * (v_target - v), v, dt);
    v = r.first;  // controller integrates to the next speed (point mass)
    overshoot = std::max(overshoot, v - v_target);
  }
  check("long closed-loop: settles to target", approx(v, v_target, 0.1));
  check("long closed-loop: overshoot < 5%", overshoot < 0.05 * v_target);
}

void test_lateral_closed_loop()
{
  // Kinematic bicycle recovering to a straight line from a lateral offset; the
  // proportional steering law is scaffolding to exercise the shaping layer in a loop.
  LateralController c;
  const double v = 10.0, lf = 2.7, dt = 0.02, ky = 0.3, kpsi = 1.0;
  double y = 1.0, psi = 0.0, max_abs_y = 0.0;
  for (int i = 0; i < 2000; ++i) {
    const double raw_steer = -(ky * y + kpsi * psi);
    const double delta = c.compute(raw_steer, dt);
    y += v * std::sin(psi) * dt;
    psi += v / lf * std::tan(delta) * dt;
    max_abs_y = std::max(max_abs_y, std::fabs(y));
  }
  check("lat closed-loop: offset recovers", std::fabs(y) < 0.05);
  check("lat closed-loop: no divergence", max_abs_y <= 1.0 + 1e-6);
}

}  // namespace

int main()
{
  test_longitudinal_unit();
  test_lateral_unit();
  test_longitudinal_closed_loop();
  test_lateral_closed_loop();
  std::printf(
    "\n%s (%d failure%s)\n", g_failures ? "FAILED" : "ALL PASS", g_failures,
    g_failures == 1 ? "" : "s");
  return g_failures == 0 ? 0 : 1;
}
