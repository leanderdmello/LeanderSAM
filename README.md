# telerob
Contains the meta and development packages for the autopilot and UI for Telerob's SLAM module

# Dev container usage
If you haven't signed in to ghcr.io, you will need to do so in order to pull the dev container images:
`docker login ghcr.io`

There are currently two different dev containers defined: `interface` and `slam`. These represent development environments for the
two deployment containers on the module. Each dev container directory contains a `ros_packages` file, which lists the ROS packages that
belong to each development environment. When a dev container is created, all `debian_depends` dependencies for the listed ROS packages are installed.

To build and run a dev container, use the `Dev Containers: Rebuild and Reopen in Container` command in VS Code. You will be prompted to select a dev container configuration. You can rebuild the container at any time by running the same command.

There are also a build task and launch configuration available for building and running the ROS nodes within the dev container. You can access these by opening the command palette and searching for `Tasks: Run Build Task` or `Debug: Start Debugging`. Note that you must be inside the dev container for these to work.

To launch the container:
`docker run -i -t --name telerob --rm -v $HOME:$HOME -e DISPLAY=$DISPLAY --gpus=all --runtime nvidia --network=host telerob-dev:latest`

To build ros packages run
`colcon build`

To run your package with ros
`source install/setup.bash`

Followed by 
`ros2 run <package_name> <executable>`
