# 1.1.1
- Fix LED indicator status during update process
- Add storage of container logs on SD card to update process when containers fail to start during update
- [internal] Fix healthcheck issues
- Improved handling of Ouster network configuration
- [internal] update process stability fixes
- Interface changes for sake of consistency
    - Change implementation to adhere to specification wrt SetRobotLocationToCameraPosition command (previously was called "SetRobotLocationToCameraLocation" in implementation)
    - Change argument name for SetVideoStreamOption to "SetVideoStreamOption" (previously was "SetVideoStreamingOption", causing confusion) in implementation and specification.
- Move floor name indicator on stream to bottom left due to overlap with other stream overlay elements
- Make default floor name indicator text empty to avoid confusion when not viewing a floor (previously default was "Floor 0", which would show briefly in some scenarios).
- Remove incorrect creation of "backup" floor.
- [internal] Devcontainer updates
- [internal] Enable persistent journald logging


# 1.1.0

- Fix internal state synchronization issues that would cause visualization issues
- Fix marker visualization issues when changing view floors
- Add bitrates to configuration file
- Update SLAM configuration file template to reflect changes made during testing

# nightly.4b06d90 (sent to Telerob customer)

- [internal] Add (toggleable) Telerob remote startup simulation to web client
- Avoid unwanted container restart on boot caused by power save handling
- Further map export viewport fixes
- Add gstreamer pipeline definition to configuration file
- Fix configuration file handling for interface nodes

# nightly.3c36125 (sent to Telerob customer)

- [WIP] Further implementation of localization functionality (still needs performance improvements and testing)
- Fix export view positioning and scaling so whole map fits in export
- Fix stream performance with threading and gstreamer pipeline tuning
    - [internal] Add profiler to map visualizer node for performance analysis

# nightly.7aaf72f (sent to Telerob customer)

- [internal] Web client improvements
    - gamepad support for ModifyCamera commands, and some other commands mapped to buttons
- Fix to offline package installs during updates
- Enforce startup order of containers to ensure system is ready for commands when Control Command Interface is available
- [internal] Fix release artifact size issues
- [WIP] Enable localization functionality (still needs performance improvements and testing)
- [internal] Add install of development tooling (tcpdump, nano)
- Config change to improve map quality (lower leaf sizes for point cloud filtering)
- Only show robot position when viewing the mapping floor

# nightly.d3a3a1d (sent to Telerob customer)

- [internal] separate development deployment
- Fix power save mode by changing behavior:
    - "stop containers + lidar power save" instead of "turn off computer"
        - fixes issue with dockers not starting on boot
    - "hold power button" to disable power save mode
        - behavior can be inverted in configuration (see below)
- More robust static network setup from configuration
- More fields in configuration file
    - power.save_on_hold: invert "hold power button" behavior (default: false)
    - stream.* options now work
    - slam.imu ("lidar" or "xsens") to select imu source for slam
- Created web client for testing of control command interface
- [internal] expose configuration file contents to containers using ROS2 parameter file templates.
    - Replaces previous method of exposing ROS2 parameter files of SLAM.
- Disable default docker network bridge to prevent incorrect routing of ipv4 link-local addresses
- Update gstreamer pipeline with explicit "baseline" h264 profile
- Add overlay for name of currently viewed floor
- Add scale indicator overlay
- Fix position+rotation of cursor
- Fix robot sizing in stream
- [internal] Telerob cli as shorthand for common development commands
    - reload-config = manual reload of changes in configuration.toml file
    - compose = docker compose command with correct arguments
    - enter = start bash session in interface or slam container
    - rebuild = build all code in containers (only available in development version)
- Adhere to interface specification for camera controls
- Add view rotation mapped to Pan parameter of ModifyCamera command
- [internal] Enable offline updates when host package installs are required
- Render robot position from odometry

# 1.0.0

Initial release

## Features:
- Floor mapping and viewing in 2D through video stream
    - Floor management (add, remove, rename)
    - View control (translation and zoom; no rotation)
    - Marker placement, removal (by index), and rename
- Export viewed map as png file, showing floor plan and markers
- Runtime video stream configuration
    - On/off switch
    - Low/high resolution mode switch
- (simple) Module configuration through configuration file on SD card
    - Static network configuration on module interface ethernet
- Module software update mechanism using update package on SD card
- [WIP] Power saving mode through GPIO from MCU (still needs testing and improvements)

## Known issues:
- Missing localization
- Missing view rotation controls
- View element issues:
    - No scale indicator, current floor name
    - Cursor position and rotation incorrect
    - Robot size incorrect
- Map export viewport issues (positioning and scaling)
- Power save implementation is incomplete, does not allow for starting in power save mode
