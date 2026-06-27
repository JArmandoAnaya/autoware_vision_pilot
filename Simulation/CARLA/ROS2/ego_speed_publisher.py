#!/usr/bin/env python3
"""Publish the CARLA ego ground speed as std_msgs/Float64 on /vehicle/speed.

VisionPilot's VehicleRos2Interface subscribes /vehicle/speed (m/s). CARLA 0.10
Shipping does not natively emit a usable ego odometry/velocity topic, so this
node reads the hero actor's velocity over the CARLA PythonAPI and publishes the
ground-speed magnitude (hypot(vx, vy) -- frame-handedness independent).

Runs where the CARLA PythonAPI is importable (CARLA install's PythonAPI/carla on
PYTHONPATH, or the carla wheel). Pair it with ros_carla_config.py, which spawns
the hero ego. Use the in-package ego_speed_republisher node instead when an
nav_msgs/Odometry source is already on the ROS2 graph.
"""
import argparse
import math
import time

import carla
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class EgoSpeedPublisher(Node):
    def __init__(self, host, port, role_name, speed_topic, rate_hz):
        super().__init__("ego_speed_publisher")
        self.client = carla.Client(host, port)
        self.client.set_timeout(60.0)
        self.world = self.client.get_world()
        self.role_name = role_name

        self.ego = None
        while self.ego is None:
            self.ego = self._find_ego()
            if self.ego is None:
                self.get_logger().warn("ego (role_name=%s) not found, waiting ..." % role_name)
                time.sleep(1.0)

        self.pub = self.create_publisher(Float64, speed_topic, 10)
        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info("publishing ego speed -> %s @ %.0f Hz" % (speed_topic, rate_hz))

    def _find_ego(self):
        for actor in self.world.get_actors().filter("vehicle.*"):
            if actor.attributes.get("role_name") == self.role_name:
                return actor
        return None

    def _tick(self):
        if self.ego is None or not self.ego.is_alive:
            return
        v = self.ego.get_velocity()
        self.pub.publish(Float64(data=math.hypot(v.x, v.y)))


def main():
    ap = argparse.ArgumentParser(description="CARLA ego speed -> /vehicle/speed")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", default=2000, type=int)
    ap.add_argument("--role-name", default="hero")
    ap.add_argument("--speed-topic", default="/vehicle/speed")
    ap.add_argument("--rate", default=20.0, type=float)
    args = ap.parse_args()

    rclpy.init()
    node = EgoSpeedPublisher(args.host, args.port, args.role_name, args.speed_topic, args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
