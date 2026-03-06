#!/bin/bash
set -e

if [ -f "/home/$USER/ws/install/setup.bash" ]; then
    echo "Sourcing workspace setup.bash"
    source "/home/$USER/ws/install/setup.bash"
else
    echo "Sourcing default setup.bash"
    source "/opt/ros/humble/setup.bash"
fi

# If no command is provided, run bash
if [ "$#" -eq "0" ]; then
    bash
else
    "$@"
fi
