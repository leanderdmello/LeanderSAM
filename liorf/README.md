
# LioRF

This is a **lifecycle node adaptation** of [LioRF (ros2 branch)](https://github.com/YJZLuckyBoy/liorf), adapted to use ROS 2 lifecycle nodes. The code is a copied from the original open source repository and is far from optimal yet.

## Note

For additional launch files and configurations, please refer to the [original LioRF repository](https://github.com/YJZLuckyBoy/liorf).
There are configurations for different that have been setup before.

## Running

Build the workspace using:

```bash
colcon build --packages-select liorf
```

Launch the main node:

```bash
ros2 launch liorf run_lio_sam_ouster_lifecycle.launch.py
```

Multiple lifecycle nodes are launched that need to be activated before SLAM will start:

```bash
ros2 lifecycle set <node_name> configure
ros2 lifecycle set <node_name> activate
```

Run a rosbag with imu and lidar data for testing:

```bash
ros2 bag play <rosbag> --remap <imu topic name> /slam/lidar/imu <lidar topic name> /slam/lidar/points
```
It is also possible to change the imu and pointcloud topic names that Liorf listens to in the configuration file.
