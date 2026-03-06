# Copyright 2025 Avular Holding B.V.
# All Rights Reserved
# You may use this code under the terms of the Avular
# Software End-User License Agreement.
#
# You should have received a copy of the Avular
# Software End-User License Agreement license with
# this file, or download it from: avular.com/eula

from launch.actions import (
    SetLaunchConfiguration,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_avular.launch_description import (
    launch_description as avular_launch_description,
)
from launch_avular.substitutions import YamlModifier


def generate_launch_description():
    ld = avular_launch_description()

    ld.add_action(SetLaunchConfiguration("ns", "/interface"))

    param_base_path = PathJoinSubstitution(
        [FindPackageShare(LaunchConfiguration("config_package")), "config"]
    )

    packages = [
        "control_command_adapter",
        "rtp_node",
        "map_manager",
        "map_visualizer",
        "network_manager",
    ]

    for package in packages:
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare(package),
                            "launch",
                            package + ".launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "config_package": "interface_bringup",
                    "config_dir": "config",
                    "params_file": PathJoinSubstitution(
                        [param_base_path, package + ".yaml"]
                    ),
                    "ns": LaunchConfiguration("ns"),
                    "respawn": "true",
                }.items(),
            )
        )

    return ld
