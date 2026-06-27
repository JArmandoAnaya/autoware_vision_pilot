"""Pure VisionPilot-actuation -> CARLA-vehicle-control mapping (no ROS2 / CARLA deps).

Kept separate from the ROS2 node so it is unit-testable anywhere with plain pytest.

VisionPilot's planner publishes the *tyre angle* (rad) and a longitudinal
*acceleration* (m/s^2). CARLA's CarlaEgoVehicleControl wants normalized
throttle/steer/brake in [0, 1] / [-1, 1]. This maps the former to the latter
directly (no inner speed loop -- the planner already closed it):

    steer  = clamp(steer_rad / max_steer, [-1, 1])
    accel  >  deadband -> throttle = clamp(throttle_gain * accel, [0, throttle_max]), brake = 0
    accel  < -deadband -> brake    = clamp(brake_gain * (-accel), [0, 1]),            throttle = 0
    otherwise          -> coast (throttle = brake = 0)
"""
from dataclasses import dataclass


@dataclass
class ControlParams:
    """Vehicle/gain parameters for the actuation->control mapping."""

    max_steer: float = 1.2217      # front-wheel steer limit [rad]; CARLA lincoln.mkz ~70 deg
    throttle_gain: float = 0.5     # throttle per +1 m/s^2 of commanded accel
    brake_gain: float = 0.5        # brake per +1 m/s^2 of commanded decel
    accel_deadband: float = 0.05   # [m/s^2] band around zero with neither throttle nor brake
    throttle_max: float = 1.0      # throttle ceiling (<=1.0 to cap aggressiveness)


def steer_to_carla(steer_rad: float, max_steer: float) -> float:
    """Tyre angle [rad] -> normalized steer [-1, 1]."""
    return max(-1.0, min(1.0, steer_rad / max_steer))


def accel_to_throttle_brake(accel_mps2: float, p: ControlParams):
    """Commanded longitudinal accel [m/s^2] -> (throttle, brake), each in [0, 1]."""
    if accel_mps2 > p.accel_deadband:
        return max(0.0, min(p.throttle_max, p.throttle_gain * accel_mps2)), 0.0
    if accel_mps2 < -p.accel_deadband:
        return 0.0, max(0.0, min(1.0, p.brake_gain * (-accel_mps2)))
    return 0.0, 0.0


def steering_accel_to_control(steer_rad: float, accel_mps2: float, p: ControlParams):
    """(tyre angle [rad], accel [m/s^2]) -> (throttle, steer, brake)."""
    throttle, brake = accel_to_throttle_brake(accel_mps2, p)
    return throttle, steer_to_carla(steer_rad, p.max_steer), brake
