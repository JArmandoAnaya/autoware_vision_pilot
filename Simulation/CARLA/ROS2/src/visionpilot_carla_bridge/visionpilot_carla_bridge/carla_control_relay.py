#!/usr/bin/env python3
"""Relay VisionPilot Float64 actuation -> carla_msgs/CarlaEgoVehicleControl.

VisionPilot (ENABLE_ROS2_INTERFACE=ON) publishes two std_msgs/Float64 commands:
  /vehicle/steering_cmd   tyre angle [rad]
  /vehicle/throttle_cmd   longitudinal acceleration [m/s^2]

CARLA 0.10 Shipping has no ros2_ackermann_control, so the ego only accepts
carla_msgs/CarlaEgoVehicleControl. This node caches the latest steering/accel and
publishes a CarlaEgoVehicleControl at a fixed rate (a steady control stream keeps
the ego responsive even if one input stalls). The numeric mapping lives in the
pure, unit-tested control_mapping module.

All topics, gains and rate are declared ROS2 parameters.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from carla_msgs.msg import CarlaEgoVehicleControl

from visionpilot_carla_bridge.control_mapping import ControlParams, steering_accel_to_control


class CarlaControlRelay(Node):
    def __init__(self):
        super().__init__("carla_control_relay")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("steering_topic", "/vehicle/steering_cmd")
        self.declare_parameter("throttle_topic", "/vehicle/throttle_cmd")
        self.declare_parameter("control_topic", "/carla/hero/vehicle_control_cmd")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("max_steer", ControlParams.max_steer)
        self.declare_parameter("throttle_gain", ControlParams.throttle_gain)
        self.declare_parameter("brake_gain", ControlParams.brake_gain)
        self.declare_parameter("accel_deadband", ControlParams.accel_deadband)
        self.declare_parameter("throttle_max", ControlParams.throttle_max)

        g = self.get_parameter
        steering_topic = g("steering_topic").value
        throttle_topic = g("throttle_topic").value
        control_topic = g("control_topic").value
        rate = float(g("control_rate_hz").value)
        self.params = ControlParams(
            max_steer=float(g("max_steer").value),
            throttle_gain=float(g("throttle_gain").value),
            brake_gain=float(g("brake_gain").value),
            accel_deadband=float(g("accel_deadband").value),
            throttle_max=float(g("throttle_max").value),
        )

        # ── State (latest command, held between updates) ──────────────────────
        self.steer_rad = 0.0
        self.accel_mps2 = 0.0

        self.create_subscription(Float64, steering_topic, self._on_steer, 10)
        self.create_subscription(Float64, throttle_topic, self._on_accel, 10)
        self.pub = self.create_publisher(CarlaEgoVehicleControl, control_topic, 10)
        self.timer = self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            "relay %s + %s -> %s @ %.0f Hz (max_steer=%.4f rad, k_thr=%.2f, k_brk=%.2f)"
            % (steering_topic, throttle_topic, control_topic, rate,
               self.params.max_steer, self.params.throttle_gain, self.params.brake_gain)
        )

    def _on_steer(self, msg: Float64):
        self.steer_rad = float(msg.data)

    def _on_accel(self, msg: Float64):
        self.accel_mps2 = float(msg.data)

    def _publish(self):
        throttle, steer, brake = steering_accel_to_control(
            self.steer_rad, self.accel_mps2, self.params
        )
        out = CarlaEgoVehicleControl()
        out.header.stamp = self.get_clock().now().to_msg()
        out.throttle = float(throttle)
        out.steer = float(steer)
        out.brake = float(brake)
        out.hand_brake = False
        out.reverse = False
        out.gear = 1
        out.manual_gear_shift = False
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CarlaControlRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
