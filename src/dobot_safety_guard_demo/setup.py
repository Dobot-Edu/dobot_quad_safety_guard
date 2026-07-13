from glob import glob
from setuptools import setup

package_name = "dobot_safety_guard_demo"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Dobot Demo Team",
    maintainer_email="support@example.com",
    description="Front/back depth obstacle safety guard demo for Dobot quadruped.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "depth_bridge_node = dobot_safety_guard_demo.depth_bridge_node:main",
            "safety_guard_node = dobot_safety_guard_demo.safety_guard_node:main",
        ],
    },
)
