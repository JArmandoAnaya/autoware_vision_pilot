"""Unit tests for the VisionPilot-actuation -> CARLA-control mapping.

Pure functions only (no ROS2 / CARLA), so this runs anywhere with pytest:
    pytest test/test_control_mapping.py
"""
import pytest

from visionpilot_carla_bridge.control_mapping import (
    ControlParams,
    accel_to_throttle_brake,
    steer_to_carla,
    steering_accel_to_control,
)

P = ControlParams()  # max_steer=1.2217, throttle_gain=0.5, brake_gain=0.5, deadband=0.05


def test_steer_normalized_by_max_steer():
    assert steer_to_carla(P.max_steer / 2.0, P.max_steer) == pytest.approx(0.5)


def test_steer_clamped_to_unit():
    assert steer_to_carla(2.0 * P.max_steer, P.max_steer) == pytest.approx(1.0)
    assert steer_to_carla(-2.0 * P.max_steer, P.max_steer) == pytest.approx(-1.0)


def test_throttle_on_positive_accel():
    # Small accel stays below the cap and unclamped.
    throttle, brake = accel_to_throttle_brake(1.0, P)
    assert throttle == pytest.approx(P.throttle_gain * 1.0)
    assert brake == 0.0


def test_throttle_clamped_to_max():
    throttle, brake = accel_to_throttle_brake(100.0, P)
    assert throttle == pytest.approx(P.throttle_max)
    assert brake == 0.0


def test_brake_on_negative_accel():
    throttle, brake = accel_to_throttle_brake(-2.0, P)
    assert throttle == 0.0
    assert brake == pytest.approx(P.brake_gain * 2.0)


def test_brake_clamped_to_unit():
    throttle, brake = accel_to_throttle_brake(-100.0, P)
    assert throttle == 0.0
    assert brake == pytest.approx(1.0)


def test_coast_inside_deadband():
    throttle, brake = accel_to_throttle_brake(0.0, P)
    assert (throttle, brake) == (0.0, 0.0)
    # Just inside the deadband on either side also coasts.
    assert accel_to_throttle_brake(P.accel_deadband * 0.5, P) == (0.0, 0.0)
    assert accel_to_throttle_brake(-P.accel_deadband * 0.5, P) == (0.0, 0.0)


def test_combined_mapping_order():
    # steering_accel_to_control returns (throttle, steer, brake).
    throttle, steer, brake = steering_accel_to_control(P.max_steer, 1.0, P)
    assert steer == pytest.approx(1.0)
    assert throttle == pytest.approx(P.throttle_gain)
    assert brake == 0.0


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
