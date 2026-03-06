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
    liorf_path = FindPackageShare(package="liorf").find(
        "liorf"
    )
    config_path = os.path.join(liorf_path, "config", "lio_sam_ouster.yaml")

    # Declare the launch arguments
    ns_arg = DeclareLaunchArgument(
        name="ns",
        default_value="slam",
        description="Namespace to be used for the launched node(s)",
    )

    static_transform_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_map_to_odom",
        namespace=LaunchConfiguration("ns"),
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],
        output="screen",
    )

    imuPreintegration_node = Node(
        package="liorf",
        executable="liorf_imuPreintegration_lifeCycle",
        namespace=LaunchConfiguration("ns"),
        parameters=[config_path],
        output="screen",
    )

    imageProjection_node = LifecycleNode(
        package="liorf",
        executable="liorf_imageProjection_lifeCycle",
        name="liorf_imageProjection",
        namespace=LaunchConfiguration("ns"),
        parameters=[config_path],
        output="screen",
    )

    mapOptimization_node = LifecycleNode(
        package="liorf",
        executable="liorf_mapOptmization_lifeCycle",
        name="liorf_mapOptimization",
        namespace=LaunchConfiguration("ns"),
        parameters=[config_path],
        output="screen",
    )

    # Create a launch description and populate
    ld = LaunchDescription()

    # Arguments
    ld.add_action(ns_arg)

    # Launch Node
    ld.add_action(imuPreintegration_node)
    ld.add_action(imageProjection_node)
    ld.add_action(mapOptimization_node)
    ld.add_action(static_transform_node)

    return ld