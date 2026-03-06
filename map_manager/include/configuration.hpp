// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#pragma once

#include <string>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"


struct Configuration
{
public:
    Configuration(rclcpp::Node &node);

    std::string                            version_file_path;
    std::string                            map_path;
    std::chrono::duration<int, std::milli> map_save_interval_ms;
    std::chrono::duration<int, std::milli> slam_mode_timeout_ms;
    std::string                            topic_robot_status;
    std::string                            topic_trajectory;
    std::string                            topic_global_map;
    std::string                            topic_visualization_map;
    std::string                            topic_point_cloud_available;
    std::string                            topic_mapping_point_cloud_available;
    std::string                            topic_trajectories;
    std::string                            topic_slam_mode;
    std::string                            srv_add_floor;
    std::string                            srv_delete_floor;
    std::string                            srv_clear_floor;
    std::string                            srv_get_floors;
    std::string                            srv_select_mapping_floor;
    std::string                            srv_get_mapping_floor;
    std::string                            srv_select_view_floor;
    std::string                            srv_get_view_floor;
    std::string                            srv_set_floor_name;
    std::string                            srv_export_map;
    std::string                            srv_clear_map;
    std::string                            srv_get_software_versions;
    std::string                            srv_clear_path;
    std::string                            srv_set_slam_mode;
};
