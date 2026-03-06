#!/usr/bin/env bash

env_name=$(cat /tmp/.env_name)
ros_packages="$(cat "$(dirname "$0")/${env_name}/ros_packages" | tr '\n' ' ')"
packages_select_arg="--packages-select ${ros_packages}"

if [ -z "$env_name" ]; then
    echo "Warning: Not in dev environment, all packages will be built, which will probably fail."
    packages_select_arg=""
fi

set +e
if [ ! -z "$ros_packages" ]; then
    echo "Building ROS packages: $ros_packages"
fi

colcon build ${packages_select_arg}

# Update /tmp/.env_executables after building
source install/setup.bash
echo -n "" > /tmp/.env_executables
for package in $ros_packages; do
    ros2 pkg executables "$package" >> /tmp/.env_executables
done

# Update /tmp/.env_launch_files after building
echo -n "" > /tmp/.env_launch_files

# First add the launch files of the bringup package of this environment
find $(ros2 pkg prefix ${env_name}_bringup)/share/${env_name}_bringup/launch -name *.launch.py >> /tmp/.env_launch_files

# Then add the launch files of all other packages in the environment
for package in $ros_packages; do
    find $(ros2 pkg prefix $package)/share/$package/launch -name *.launch.py >> /tmp/.env_launch_files || true
done
