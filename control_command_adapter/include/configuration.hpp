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
#include <rclcpp/rclcpp.hpp>

struct Configuration
{
public:
    Configuration(rclcpp::Node::SharedPtr node);

    int                                    port_;
    std::chrono::duration<int, std::milli> cmd_timeout_;
    std::string                            modify_camera_service_;
    std::string                            move_camera_to_robot_service_;
    std::string                            follow_robot_service_;
    std::string                            set_robot_location_to_camera_service_;
    std::string                            add_floor_service_;
    std::string                            delete_floor_service_;
    std::string                            clear_floor_service_;
    std::string                            get_floors_service_;
    std::string                            select_mapping_floor_service_;
    std::string                            get_mapping_floor_service_;
    std::string                            select_view_floor_service_;
    std::string                            get_view_floor_service_;
    std::string                            set_floor_name_service_;
    std::string                            export_map_service_;
    std::string                            clear_map_service_;
    std::string                            add_marker_service_;
    std::string                            remove_marker_service_;
    std::string                            get_markers_service_;
    std::string                            set_marker_label_service_;
    std::string                            set_marker_label_at_position_service_;
    std::string                            get_software_versions_service_;
    std::string                            scale_unit_meters_service_;
    std::string                            set_video_resolution_mode_service_;
    std::string                            set_video_stream_address_service_;
    std::string                            set_video_stream_enabled_service_;
    std::string                            clear_path_service_;
    std::string                            robot_status_topic_;
};
