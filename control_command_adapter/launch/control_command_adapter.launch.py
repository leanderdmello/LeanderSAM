# Copyright 2025 Avular Holding B.V.
# All Rights Reserved
# You may use this code under the terms of the Avular
# Software End-User License Agreement.
#
# You should have received a copy of the Avular
# Software End-User License Agreement license with
# this file, or download it from: avular.com/eula

from launch import LaunchDescription
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    NODE_NAME = "control_command_adapter"

    override_param_path = f"/var/opt/telerob/overrides/{NODE_NAME}.yaml"

    param_paths = [
        PathJoinSubstitution(
            [
                FindPackageShare(LaunchConfiguration("config_package")),
                LaunchConfiguration("config_dir"),
                LaunchConfiguration("params_file"),
            ]
        )
    ]

    if os.path.exists(override_param_path):
        param_paths.append(override_param_path)

    ld = LaunchDescription()

    ld.add_action(
        GroupAction(
            [
                DeclareLaunchArgument(
                    "use_sim_time",
                    default_value="false",
                    description="Use simulation time",
                ),
                DeclareLaunchArgument(
                    "config_package",
                    default_value=NODE_NAME,
                    description="Package containing the config files",
                ),
                DeclareLaunchArgument(
                    "config_dir",
                    default_value="config",
                    description="Directory inside package containing the config files",
                ),
                DeclareLaunchArgument(
                    "params_file",
                    default_value=NODE_NAME + ".yaml",
                    description="Filename of the parameter file",
                ),
                DeclareLaunchArgument(
                    "ns",
                    default_value="",
                    description="Namespace to be used for the launched node(s)",
                ),
                DeclareLaunchArgument(
                    "suffix",
                    default_value="",
                    description="Suffix to be added to node name",
                ),
                DeclareLaunchArgument(
                    "output",
                    default_value="log",
                    description="Whether to output to `log` or `screen`",
                ),
                DeclareLaunchArgument(
                    "respawn",
                    default_value="false",
                    description="Whether to respawn the node if it dies",
                ),
                SetParameter(
                    name="use_sim_time", value=LaunchConfiguration("use_sim_time")
                ),
                Node(
                    package=NODE_NAME,
                    namespace=LaunchConfiguration("ns"),
                    executable=NODE_NAME,
                    name=NODE_NAME,
                    output=LaunchConfiguration("output"),
                    parameters=param_paths,
                    respawn=PythonExpression(
                        ["'", LaunchConfiguration("respawn"), "' == 'true'"]
                    ),
                    respawn_delay=1.0,
                ),
            ]
        )
    )

    return ld
