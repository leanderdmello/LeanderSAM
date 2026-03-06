// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include "configuration.hpp"

// ROS2 Core
#include <rclcpp/rclcpp.hpp>

Configuration::Configuration(rclcpp::Node::SharedPtr node)
{
    // Declare parameters with sensible defaults so get_parameter() below won't fail
    node->declare_parameter("port", 9090);
    node->declare_parameter("cmd_timeout", 10000);
    node->declare_parameter("services.client.modify_camera", "modify_camera");
    node->declare_parameter("services.client.move_camera_to_robot", "move_camera_to_robot");
    node->declare_parameter("services.client.follow_robot", "follow_robot");
    node->declare_parameter("services.client.set_robot_location_to_camera",
                            "set_robot_location_to_camera");
    node->declare_parameter("services.client.add_floor", "add_floor");
    node->declare_parameter("services.client.delete_floor", "delete_floor");
    node->declare_parameter("services.client.clear_floor", "clear_floor");
    node->declare_parameter("services.client.get_floors", "get_floors");
    node->declare_parameter("services.client.select_mapping_floor", "select_mapping_floor");
    node->declare_parameter("services.client.get_mapping_floor", "get_mapping_floor");
    node->declare_parameter("services.client.select_view_floor", "select_view_floor");
    node->declare_parameter("services.client.get_view_floor", "get_view_floor");
    node->declare_parameter("services.client.set_floor_name", "set_floor_name");
    node->declare_parameter("services.client.export_map", "export_map");
    node->declare_parameter("services.client.clear_map", "clear_map");
    node->declare_parameter("services.client.add_marker", "add_marker");
    node->declare_parameter("services.client.remove_marker", "remove_marker");
    node->declare_parameter("services.client.get_markers", "get_markers");
    node->declare_parameter("services.client.set_marker_label", "set_marker_label");
    node->declare_parameter("services.client.set_marker_label_at_position", "set_marker_label_at");
    node->declare_parameter("services.client.get_software_versions", "get_software_versions");
    node->declare_parameter("services.client.scale_unit_meters", "scale_unit_meters");
    node->declare_parameter("services.client.set_video_resolution_mode",
                            "set_video_resolution_mode");
    node->declare_parameter("services.client.set_video_stream_address", "set_video_stream_address");
    node->declare_parameter("services.client.set_video_stream_enabled", "set_video_stream");
    node->declare_parameter("services.client.clear_path", "clear_path");
    node->declare_parameter("topics.out.robot_status", "robot_status");

    port_ = node->get_parameter("port").as_int();
    cmd_timeout_ =
        std::chrono::duration<int, std::milli>{node->get_parameter("cmd_timeout").as_int()};

    modify_camera_service_ = node->get_parameter("services.client.modify_camera").as_string();
    move_camera_to_robot_service_ =
        node->get_parameter("services.client.move_camera_to_robot").as_string();
    follow_robot_service_ = node->get_parameter("services.client.follow_robot").as_string();
    set_robot_location_to_camera_service_ =
        node->get_parameter("services.client.set_robot_location_to_camera").as_string();
    add_floor_service_    = node->get_parameter("services.client.add_floor").as_string();
    delete_floor_service_ = node->get_parameter("services.client.delete_floor").as_string();
    clear_floor_service_  = node->get_parameter("services.client.clear_floor").as_string();
    get_floors_service_   = node->get_parameter("services.client.get_floors").as_string();
    select_mapping_floor_service_ =
        node->get_parameter("services.client.select_mapping_floor").as_string();
    get_mapping_floor_service_ =
        node->get_parameter("services.client.get_mapping_floor").as_string();
    select_view_floor_service_ =
        node->get_parameter("services.client.select_view_floor").as_string();
    get_view_floor_service_   = node->get_parameter("services.client.get_view_floor").as_string();
    set_floor_name_service_   = node->get_parameter("services.client.set_floor_name").as_string();
    export_map_service_       = node->get_parameter("services.client.export_map").as_string();
    clear_map_service_        = node->get_parameter("services.client.clear_map").as_string();
    add_marker_service_       = node->get_parameter("services.client.add_marker").as_string();
    remove_marker_service_    = node->get_parameter("services.client.remove_marker").as_string();
    get_markers_service_      = node->get_parameter("services.client.get_markers").as_string();
    set_marker_label_service_ = node->get_parameter("services.client.set_marker_label").as_string();
    set_marker_label_at_position_service_ =
        node->get_parameter("services.client.set_marker_label_at_position").as_string();
    get_software_versions_service_ =
        node->get_parameter("services.client.get_software_versions").as_string();
    scale_unit_meters_service_ =
        node->get_parameter("services.client.scale_unit_meters").as_string();
    set_video_resolution_mode_service_ =
        node->get_parameter("services.client.set_video_resolution_mode").as_string();
    set_video_stream_address_service_ =
        node->get_parameter("services.client.set_video_stream_address").as_string();
    set_video_stream_enabled_service_ =
        node->get_parameter("services.client.set_video_stream_enabled").as_string();
    clear_path_service_ = node->get_parameter("services.client.clear_path").as_string();
    robot_status_topic_ = node->get_parameter("topics.out.robot_status").as_string();
}
