// Deterministic conversion tests for DbwAdapter through a fake sink. No #308, no panda.
#include <control/control_command.hpp>
#include <control/dbw_adapter.hpp>
#include <control/dbw_sink.hpp>

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

// Records the last send(...) so tests can inspect the converted DBW values.
class FakeDbwSink : public IDbwSink
{
public:
  bool send(
    double steering_deg, double accelerator_pos, bool brake, double speed_kmh,
    double wheel_kmh) override
  {
    steering_deg_ = steering_deg;
    accelerator_pos_ = accelerator_pos;
    brake_ = brake;
    speed_kmh_ = speed_kmh;
    wheel_kmh_ = wheel_kmh;
    ++calls_;
    return true;
  }

  double steering_deg_ = 0.0;
  double accelerator_pos_ = 0.0;
  bool brake_ = false;
  double speed_kmh_ = 0.0;
  double wheel_kmh_ = 0.0;
  int calls_ = 0;
};

void test_unit_conversions()
{
  {  // rad -> deg and the call reaches the sink
    FakeDbwSink sink;
    DbwAdapter adapter(sink);
    bool ok = adapter.send({M_PI / 4.0, 0.0, 0.0}, 0.0);
    check(
      "rad->deg (pi/4 -> 45)", ok && sink.calls_ == 1 && approx(sink.steering_deg_, 45.0, 1e-9));
  }
  {  // steering sign preserved
    FakeDbwSink sink;
    DbwAdapter adapter(sink);
    adapter.send({-M_PI / 6.0, 0.0, 0.0}, 0.0);
    check(
      "steering sign preserved",
      sink.steering_deg_ < 0.0 && approx(sink.steering_deg_, -30.0, 1e-9));
  }
  {  // m/s -> km/h for both target speed and ego wheel speed
    FakeDbwSink sink;
    DbwAdapter adapter(sink);
    adapter.send({0.0, 10.0, 0.0}, 5.0);
    check(
      "m/s->km/h (speed + wheel)",
      approx(sink.speed_kmh_, 36.0, 1e-9) && approx(sink.wheel_kmh_, 18.0, 1e-9));
  }
}

void test_accel_to_pedal()
{
  {  // full a_max -> 100%, no brake
    FakeDbwSink sink;
    DbwAdapter adapter(sink);  // default a_max = 1.5
    adapter.send({0.0, 0.0, 1.5}, 0.0);
    check(
      "accel a_max -> 100% accelerator",
      !sink.brake_ && approx(sink.accelerator_pos_, 100.0, 1e-9));
  }
  {  // half a_max -> 50%
    FakeDbwSink sink;
    DbwAdapter adapter(sink);
    adapter.send({0.0, 0.0, 0.75}, 0.0);
    check(
      "accel a_max/2 -> 50% accelerator",
      !sink.brake_ && approx(sink.accelerator_pos_, 50.0, 1e-9));
  }
  {  // negative accel -> brake, accelerator zero
    FakeDbwSink sink;
    DbwAdapter adapter(sink);
    adapter.send({0.0, 0.0, -2.0}, 0.0);
    check(
      "negative accel -> brake, 0% accelerator",
      sink.brake_ && approx(sink.accelerator_pos_, 0.0, 1e-9));
  }
}

void test_clamps()
{
  {  // steering beyond +/-540 deg is clamped
    FakeDbwSink sink;
    DbwAdapter adapter(sink);
    adapter.send({10.0, 0.0, 0.0}, 0.0);  // 10 rad ~= 573 deg
    check("steering clamped to +540", approx(sink.steering_deg_, 540.0, 1e-9));
    adapter.send({-10.0, 0.0, 0.0}, 0.0);
    check("steering clamped to -540", approx(sink.steering_deg_, -540.0, 1e-9));
  }
  {  // speed beyond 240 km/h is clamped (both target and wheel)
    FakeDbwSink sink;
    DbwAdapter adapter(sink);
    adapter.send({0.0, 100.0, 0.0}, 100.0);  // 360 km/h
    check(
      "speed clamped to 240",
      approx(sink.speed_kmh_, 240.0, 1e-9) && approx(sink.wheel_kmh_, 240.0, 1e-9));
  }
  {  // accelerator never exceeds 100% even for huge accel
    FakeDbwSink sink;
    DbwAdapter adapter(sink);
    adapter.send({0.0, 0.0, 100.0}, 0.0);
    check("accelerator capped at 100%", approx(sink.accelerator_pos_, 100.0, 1e-9));
  }
}

}  // namespace

int main()
{
  test_unit_conversions();
  test_accel_to_pedal();
  test_clamps();
  std::printf(
    "\n%s (%d failure%s)\n", g_failures ? "FAILED" : "ALL PASS", g_failures,
    g_failures == 1 ? "" : "s");
  return g_failures == 0 ? 0 : 1;
}
