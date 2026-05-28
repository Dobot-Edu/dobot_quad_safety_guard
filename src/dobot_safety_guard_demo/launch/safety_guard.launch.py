#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("dobot_safety_guard_demo")
    default_config = os.path.join(pkg_share, "config", "safety_guard_config.yaml")
    default_rviz = os.path.join(pkg_share, "rviz", "safety_guard.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="安全演示 YAML 配置文件路径。",
            ),
            DeclareLaunchArgument(
                "grpc_addr",
                default_value="192.168.5.2:50051",
                description="机器人 gRPC 地址。",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="是否使用演示配置启动 RViz2。",
            ),
            ExecuteProcess(
                cmd=[
                    "python",
                    "-m",
                    "dobot_safety_guard_demo.depth_bridge_node",
                    "--config",
                    LaunchConfiguration("config_file"),
                ],
                name="depth_bridge_node",
                output="screen",
            ),
            ExecuteProcess(
                cmd=[
                    "python",
                    "-m",
                    "dobot_safety_guard_demo.safety_guard_node",
                    "--config",
                    LaunchConfiguration("config_file"),
                    "--grpc-addr",
                    LaunchConfiguration("grpc_addr"),
                ],
                name="safety_guard_node",
                output="screen",
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "tf2_ros",
                    "static_transform_publisher",
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "0",
                    "--frame-id",
                    "safety_guard_base",
                    "--child-frame-id",
                    "front_depth_camera",
                ],
                name="base_to_front_static_tf",
                output="screen",
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "tf2_ros",
                    "static_transform_publisher",
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "3.1415926",
                    "--frame-id",
                    "safety_guard_base",
                    "--child-frame-id",
                    "back_depth_camera",
                ],
                name="base_to_back_static_tf",
                output="screen",
            ),
            ExecuteProcess(
                cmd=["rviz2", "-d", default_rviz],
                name="rviz2",
                output="screen",
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
