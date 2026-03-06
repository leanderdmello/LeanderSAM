# Copyright 2024 Avular B.V.
# All Rights Reserved
# You may use this code under the terms of the Avular
# Software End-User License Agreement.
#
# You should have received a copy of the Avular
# Software End-User License Agreement license with
# this file, or download it from: avular.com/eula

import os
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import LifecycleNode, Node
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
)


def generate_launch_description():
    # Constants for paths to different files and folders
    slam_bringup_path = FindPackageShare(package="slam_bringup").find(
        "slam_bringup"
    )
    config_path = os.path.join(slam_bringup_path, "config", "ouster.yaml")

    # Declare the launch arguments
    ns_arg = DeclareLaunchArgument(
        name="ns",
        default_value="slam",
        description="Namespace to be used for the launched node(s)",
    )

    # Because the ouster driver is a lifecycle node, and no lifecycle manager is active in the robot stack, we transition the node to active on startup.
    os_driver = LifecycleNode(
        package="ouster_ros",
        executable="os_driver",
        name="ouster_ros",
        namespace=LaunchConfiguration("ns"),
        parameters=[
            config_path,
            {"mask_path": os.path.join(slam_bringup_path, "config", "ouster_mask.png")},
        ],
        output="screen",
        respawn=True,
        respawn_delay=1.0,
        remappings=[
            ("imu", "lidar/imu"),
            ("points", "lidar/points"),
            ("metadata", "lidar/metadata"),
            ("get_metadata", "lidar/metadata"),
            ("get_config", "lidar/get_config"),
            ("set_config", "lidar/set_config"),
            ("reset", "lidar/reset"),
        ],
    )

    # Create a launch description and populate
    ld = LaunchDescription()

    # Arguments
    ld.add_action(ns_arg)

    # Launch Node
    ld.add_action(os_driver)

    return ld
