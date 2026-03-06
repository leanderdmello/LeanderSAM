#!/usr/bin/env bash

env_name="$1"
if [ -z "$env_name" ]; then
    echo "Usage: $0 <env_name>"
    exit 1
fi

echo $env_name > /tmp/.env_name

if [ ! -f "$(dirname "$0")/${env_name}/ros_packages" ]; then
    echo "Error: ros_packages file not found for environment '$env_name'"
    exit 1
fi

set +e
ros_packages="$(cat "$(dirname "$0")/${env_name}/ros_packages" | tr '\n' ' ')"
echo "${ros_packages}" > /tmp/.env_packages

# Make sure all debian dependencies are installed (most relevant when adding new dependencies)
apt_dependencies=$(get-dependencies --dir . --prefix telerob- --print --print-version --separator "=" ${ros_packages}  2>/dev/null | tr '\n' ' ')

if [ ! -z "${apt_dependencies}" ]; then
    echo "Installing debian dependencies: ${apt_dependencies}"
    sudo apt-get update && sudo apt-get install -y ${apt_dependencies}
fi

# Also install python dependencies with pip
pip_dependencies=$(get-dependencies --dir . --prefix telerob- --tags "pip_depend" --print --separator " " ${ros_packages} 2>/dev/null | tr '\n' ' ')
if [ ! -z "${pip_dependencies}" ]; then
    echo "Installing pip dependencies: ${pip_dependencies}"
    # Use separate command for each package, to continue installing even if one fails
    for pkg in ${pip_dependencies}; do
        # Try installing from Avular's private PyPI first, fallback to default PyPI if that fails
        pip install ${pkg} -i https://packages.avular.dev/pypi/avular-pypi/simple || pip install ${pkg}
    done
fi

# Add conditional sourcing of the environment setup.bash to the user's .bashrc if not already present
bashrc_entry='[ -f /workspaces/telerob/install/setup.bash ] && source /workspaces/telerob/install/setup.bash'
if ! grep -Fxq "$bashrc_entry" ~/.bashrc; then
    echo "$bashrc_entry" >> ~/.bashrc
fi
