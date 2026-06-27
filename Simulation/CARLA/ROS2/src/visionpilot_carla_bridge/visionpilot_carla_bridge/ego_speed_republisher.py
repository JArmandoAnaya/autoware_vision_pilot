#!/usr/bin/env python3
"""Republish a CARLA ego odometry topic as the scalar ego speed VisionPilot expects.

VisionPilot subscribes std_msgs/Float64 on /vehicle/speed (ground speed, m/s). When
CARLA (or the PythonAPI odom publisher) emits nav_msgs/Odometry for the ego, this
node converts it: speed = hypot(twist.linear.x, twist.linear.y) -- frame-invariant,
so the LH/RH handedness of the odometry frame does not matter.

Use this when an Odometry source is already on the graph. If CARLA emits no usable
ego state, run carla_tools/ego_speed_publisher.py (CARLA PythonAPI) instead, which
publishes /vehicle/speed directly.
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


class EgoSpeedRepublisher(Node):
    def __init__(self):
        super().__init__("ego_speed_republisher")
        self.declare_parameter("odom_topic", "/carla/hero/odometry")
        self.declare_parameter("speed_topic", "/vehicle/speed")
        odom_topic = self.get_parameter("odom_topic").value
        speed_topic = self.get_parameter("speed_topic").value

        self.pub = self.create_publisher(Float64, speed_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.get_logger().info("republishing speed: %s -> %s" % (odom_topic, speed_topic))

    def _on_odom(self, msg: Odometry):
        v = msg.twist.twist.linear
        self.pub.publish(Float64(data=math.hypot(v.x, v.y)))


def main(args=None):
    rclpy.init(args=args)
    node = EgoSpeedRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
