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
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    # Constants for paths to different files and folders
    slam_bringup_path = FindPackageShare(package="slam_bringup").find("slam_bringup")

    config_path = os.path.join(slam_bringup_path, "config", "lio_sam_ouster.yaml")
    override_config_path = f"/var/opt/telerob/overrides/lio_sam_ouster.yaml"
    if os.path.exists(override_config_path):
        config_path = override_config_path

    # Declare the launch arguments
    ns_arg = DeclareLaunchArgument(
        name="ns",
        default_value="slam",
        description="Namespace to be used for the launched node(s)",
    )

    imuPreintegration_node = Node(
        package="liorf",
        executable="liorf_imuPreintegration_lifeCycle",
        namespace=LaunchConfiguration("ns"),
        parameters=[config_path],
        output="screen",
        respawn=True,
        respawn_delay=1.0,
    )

    imageProjection_node = LifecycleNode(
        package="liorf",
        executable="liorf_imageProjection_lifeCycle",
        name="liorf_imageProjection",
        namespace=LaunchConfiguration("ns"),
        parameters=[config_path],
        output="screen",
        respawn=True,
        respawn_delay=1.0,
    )

    mapOptimization_node = LifecycleNode(
        package="liorf",
        executable="liorf_mapOptmization_lifeCycle",
        name="liorf_mapOptimization",
        namespace=LaunchConfiguration("ns"),
        parameters=[config_path],
        output="screen",
        respawn=True,
        respawn_delay=1.0,
    )

    # Create a launch description and populate
    ld = LaunchDescription()

    # Arguments
    ld.add_action(ns_arg)

    # Launch Node
    ld.add_action(imuPreintegration_node)
    ld.add_action(imageProjection_node)
    ld.add_action(mapOptimization_node)

    return ld
