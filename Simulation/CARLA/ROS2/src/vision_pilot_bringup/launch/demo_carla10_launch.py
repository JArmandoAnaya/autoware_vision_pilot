from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node

import os


def generate_launch_description():
    # .../src/carla_bridge_bringup/launch/<this file>  ->  ROS2 workspace root
    install_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    ws_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(install_dir))))
    script_path = os.path.abspath(os.path.join(ws_dir, 'ros_carla_config.py'))
    config_json = os.path.join(ws_dir, 'config', 'VisionPilot_carla10.json')

    # The new VisionPilot driver is the single C++ binary built from VisionPilot/
    # (perception -> fusion -> MPC -> control -> Ackermann). It is NOT a ROS2
    # package, so point these at your build output and CARLA config:
    #   export VISIONPILOT_BIN=/path/to/VisionPilot/build/VisionPilot
    #   export VISIONPILOT_CONFIG=/path/to/vision_pilot.conf   (from
    #       VisionPilot/config/vision_pilot_carla.conf.example)
    vp_bin = os.environ.get('VISIONPILOT_BIN', 'VisionPilot')
    vp_conf = os.environ.get('VISIONPILOT_CONFIG', 'config/vision_pilot.conf')

    return LaunchDescription([
        # CARLA native ROS2 (Fast-DDS) defaults to shared-memory transport, which does
        # not cross a host<->container boundary. Force UDP so camera data and the
        # Ackermann command actually flow. The CARLA *server* must ALSO run with
        # FASTDDS_BUILTIN_TRANSPORTS=UDPv4 (see ROS2/README.md).
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),

        # 1) Spawn ego + sensors (main_cam with enable_for_ros, ros2_ackermann_control).
        ExecuteProcess(
            cmd=['python3', script_path, '-f', config_json],
            output='screen'
        ),

        # 2) VisionPilot single-binary driver: subscribes to the camera, publishes the
        #    Ackermann command. Control lives inside the binary — there is no Python
        #    steering/longitudinal/PATHFINDER/control-relay chain anymore.
        ExecuteProcess(
            cmd=[vp_bin, '--config', vp_conf],
            output='screen'
        ),

        # 3) Optional ego-odometry fallback (publishes /hero/odom) for when CARLA's
        #    native odometry is unavailable — point control.vehicle_state_topic at it.
        Node(
            package='odom_publisher',
            executable='pub_odom_node',
            name='pub_odom_node',
            output='screen'
        ),

        # tf: ego base -> front camera frame.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='front_to_base_broadcaster',
            arguments=["1.425", "0", "0", "0", "0", "0", "hero", "hero_front"]
        ),
    ])
