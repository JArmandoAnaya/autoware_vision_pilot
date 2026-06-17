#ifndef CONTROL__DBW_ADAPTER_HPP_
#define CONTROL__DBW_ADAPTER_HPP_

#include <control/control_command.hpp>
#include <control/dbw_sink.hpp>

// Converts the agnostic SI ControlCommand into the DBW contract (degrees, accelerator
// percent + brake, km/h) and forwards it to an IDbwSink. This is the ONLY unit-, pedal-,
// and degree-aware code; it knows nothing about PR #308 / libpanda / ROS2. The accel ->
// accelerator-percent map is a placeholder pending the #308 author (findings doc 7.2).
class DbwAdapter
{
public:
  struct Config
  {
    double a_max = 1.5;            // m/s^2 mapped to 100% accelerator (placeholder, see 7.2)
    double max_steer_deg = 540.0;  // DBW steering-angle limit (#308)
    double max_speed_kmh = 240.0;  // DBW speed limit (#308)
  };

  explicit DbwAdapter(IDbwSink & sink) : sink_(sink) {}
  DbwAdapter(IDbwSink & sink, const Config & config) : sink_(sink), config_(config) {}

  // cmd: SI setpoints from the controllers. ego_v_mps: measured ego speed -> wheel km/h.
  bool send(const ControlCommand & cmd, double ego_v_mps);

  const Config & config() const { return config_; }

private:
  IDbwSink & sink_;
  Config config_;
};

#endif  // CONTROL__DBW_ADAPTER_HPP_
