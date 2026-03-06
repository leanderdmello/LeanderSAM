# Twilight Origin Autopilot

Metapackage for starting the TOM's SLAM and bringup functionalities

## Getting started

The default launch configuration to start is:

```
ros2 launch telerob_slam_bringup default.launch.py
```
This will start up the system and initialize it after an initial pose of the system has been made available.


To only launch the xsens node:
```
ros2 launch telerob_slam_bringup xsens_mti_node.launch.py
```
