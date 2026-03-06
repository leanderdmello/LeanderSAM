# Copyright 2025 Avular B.V.
# All Rights Reserved
# You may use this code under the terms of the Avular
# Software End-User License Agreement.
#
# You should have received a copy of the Avular
# Software End-User License Agreement license with
# this file, or download it from: avular.com/eula

import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    ns_arg = DeclareLaunchArgument(
        name="ns",
        default_value="slam",
        description="Namespace to be used for the launched node(s)",
    )
    # Set environment variables to control logging behavior
    ld.add_action(SetEnvironmentVariable("RCUTILS_LOGGING_USE_STDOUT", "1"))
    ld.add_action(SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"))
    ld.add_action(ns_arg)

    slam_bringup_path = FindPackageShare(package="slam_bringup").find("slam_bringup")

    parameters_file_path = os.path.join(
        slam_bringup_path, "config", "xsens_mti_node.yaml"
    )
    xsens_mti_node = Node(
        package="xsens_mti_ros2_driver",
        namespace=LaunchConfiguration("ns"),
        executable="xsens_mti_node",
        name="xsens_mti_node",
        output="screen",
        parameters=[parameters_file_path],
        arguments=[],
        remappings=[
            ("imu/data", "xsens/imu"),
        ],
        respawn=True,
        respawn_delay=1.0,
    )
    ld.add_action(xsens_mti_node)

    return ld
