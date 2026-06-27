"""Launch the VisionPilot<->CARLA bridge nodes.

  carla_control_relay    /vehicle/{steering,throttle}_cmd -> CarlaEgoVehicleControl
  ego_speed_republisher  /carla/hero/odometry -> /vehicle/speed   (Odometry source on graph)

Drop ego_speed_republisher (set use_odom_republisher:=false) when ego speed comes from
the CARLA-PythonAPI publisher (carla_tools/ego_speed_publisher.py) instead.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_odom_republisher = LaunchConfiguration("use_odom_republisher")
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_odom_republisher", default_value="true",
            description="Republish /carla/hero/odometry -> /vehicle/speed."),
        Node(
            package="visionpilot_carla_bridge",
            executable="carla_control_relay",
            name="carla_control_relay",
            output="screen",
        ),
        Node(
            package="visionpilot_carla_bridge",
            executable="ego_speed_republisher",
            name="ego_speed_republisher",
            output="screen",
            condition=IfCondition(use_odom_republisher),
        ),
    ])
