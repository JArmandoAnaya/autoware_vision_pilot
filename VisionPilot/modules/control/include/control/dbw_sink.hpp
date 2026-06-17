#ifndef CONTROL__DBW_SINK_HPP_
#define CONTROL__DBW_SINK_HPP_

// Abstract DBW transport. PR #308's CanWriter is one implementation (Phase 3b);
// a fake (tests) and a ROS2/Ackermann path are others. Units match #308's send_command.
class IDbwSink
{
public:
  virtual ~IDbwSink() = default;

  // steering: degrees; accelerator_pos: 0..100 %; speed/wheel: km/h.
  virtual bool send(
    double steering_deg, double accelerator_pos, bool brake, double speed_kmh,
    double wheel_kmh) = 0;
};

#endif  // CONTROL__DBW_SINK_HPP_
