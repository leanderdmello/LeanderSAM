# Copyright 2023 Avular Holding B.V.
# All Rights Reserved
# You may use this code under the terms of the Avular
# Software End-User License Agreement.
#
# You should have received a copy of the Avular
# Software End-User License Agreement license with
# this file, or download it from: avular.com/eula


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generates the default way to launch this node

    This launch description is set up in such a way that it can easily be included and adjusted from other launch files.
    The reason for this launch file is described in the main README.md.

    To add another launch argument specific for this node, you need to:
    * Add a ld.add_action(DeclareLaunchArgument('<ARGUMENT_NAME>',default_value=<DEFAULT_VALUE>))
    * Use the LaunchConfiguration inside the ld.add_action(Node([...]))

    You cannot use it outside of the LaunchDescription, as it will only resolve once launched.
    So, this is why you also cannot use functions like os.path.join().
    Instead, you have to use FindPackageShare() to get the share_dir and
    PathJoinSubstitution() to generate the path

    In this launch file, config is used instead of params, as there might also be some other configuration files included in the config directory, instead of just ros parameter files
    """
    # Default will be package name
    NODE_NAME = "slam_manager"

    # Find the parameter path
    param_path = PathJoinSubstitution(
        [
            FindPackageShare(LaunchConfiguration("config_package")),
            LaunchConfiguration("config_dir"),
            LaunchConfiguration("params_file"),
        ]
    )

    # We group all actions here with a GroupAction to avoid conflicts between included launch files
    # As the LaunchArguments are mostly the same for each of these launch files, we have to scope them per included launch file
    # The best way to do it, is to use a GroupAction here.
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
                SetParameter(
                    name="use_sim_time", value=LaunchConfiguration("use_sim_time")
                ),
                # Node also has to be included in the same group as the launch arguments
                Node(
                    package=NODE_NAME,
                    namespace=LaunchConfiguration("ns"),
                    executable=NODE_NAME,
                    name={NODE_NAME, LaunchConfiguration("suffix")},
                    output=LaunchConfiguration("output"),
                    parameters=[param_path],
                    respawn=True,
                    respawn_delay=1.0,
                ),
            ]
        )
    )
    return ld
