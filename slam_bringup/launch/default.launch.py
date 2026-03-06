# Copyright 2025 Avular B.V.
# All Rights Reserved
# You may use this code under the terms of the Avular
# Software End-User License Agreement.
#
# You should have received a copy of the Avular
# Software End-User License Agreement license with
# this file, or download it from: avular.com/eula

import os
from launch.actions import SetLaunchConfiguration, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_avular.substitutions import YamlModifier
from launch_ros.actions import Node
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare

from launch_avular.launch_description import (
    launch_description as avular_launch_description,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
import autopilot_ordination.node_management as node_management
import autopilot_perception.preprocessing as preprocessing


def generate_launch_description():    
    
    ld = avular_launch_description()
    ld.add_action(SetLaunchConfiguration("params_dir", "config"))
    ld.add_action(SetLaunchConfiguration("params_pkg", "slam_bringup"))
    ld.add_action(SetLaunchConfiguration("ns", "/slam"))

    # tf tree ouster to base
    # ld.add_action(
    #     static_transform("ouster_base", "os_lidar", x=0.0, y=0.0, z=0.0)
    # )

    # path to meta package
    bringup_path = FindPackageShare(package="slam_bringup").find(
        "slam_bringup"
    )

    # Start xsens driver
    xsens_mti = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_path, "launch", "xsens_mti_node.launch.py")
        ),
        launch_arguments={
            "ns": LaunchConfiguration("ns"),
        }.items(),
    )
    ld.add_action(xsens_mti)
    
    # Start Ouster driver
    ouster_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_path, "launch", "ouster.launch.py")
        ),
        launch_arguments={
            "ns": LaunchConfiguration("ns"),
        }.items(),
    )
    ld.add_action(ouster_node)
    
    # Start Liorf
    liorf_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_path, "launch", "run_lio_sam_ouster_lifecycle.launch.py")
        ),
        launch_arguments={
            "ns": LaunchConfiguration("ns"),
        }.items(),
    )
    ld.add_action(liorf_node)

    # Start Slam manager
    slammngr_launch_path = FindPackageShare(package="slam_manager").find(
        "slam_manager"
    )
    slammngr_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slammngr_launch_path, "launch", "slam_manager.launch.py")
        ),
        launch_arguments={
            "config_package": "slam_bringup",            
            "ns": LaunchConfiguration("ns"),
        }.items(),
    )
    ld.add_action(slammngr_node)
    
    node_management.launch_lifecycle_manager(
        ld,
        params_package="slam_bringup",
        state_config_filename="lifecycle_states.json",
        namespace=LaunchConfiguration("ns"),
        managed_namespace=LaunchConfiguration("ns"),
        delay=0.0,
    )

    return ld

def static_transform(
    parent: str,
    child: str,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    roll: float = 0.0,
    pitch: float = 0.0,
    yaw: float = 0.0,
):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"static_tf_{parent}_to_{child}",
        arguments=[
            "--x",
            str(x),
            "--y",
            str(y),
            "--z",
            str(z),
            "--roll",
            str(roll),
            "--pitch",
            str(pitch),
            "--yaw",
            str(yaw),
            "--frame-id",
            parent,
            "--child-frame-id",
            child,
        ],
        output="log",
    )