from glob import glob

from setuptools import find_packages, setup

package_name = "visionpilot_carla_bridge"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="visionpilot",
    maintainer_email="noreply@example.com",
    description="VisionPilot <-> CARLA 0.10 ROS2 actuation/state bridge.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "carla_control_relay = visionpilot_carla_bridge.carla_control_relay:main",
            "ego_speed_republisher = visionpilot_carla_bridge.ego_speed_republisher:main",
        ],
    },
)
