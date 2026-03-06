// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include "configuration.hpp"

Configuration::Configuration(rclcpp::Node &node)
{
    // Declare parameters with defaults matching the YAML
    node.declare_parameter("version_file_path", "/home/avular/internal_storage/versions.txt");
    node.declare_parameter("map_path", "/home/avular/internal_storage/maps/");
    node.declare_parameter("map_save_interval", 10000);
    node.declare_parameter("slam_timeout", 10000);

    // subscriptions
    node.declare_parameter("topics.subscriptions.robot_status", "robot_status");
    node.declare_parameter("topics.subscriptions.trajectory", "/slam/mapping/path");
    node.declare_parameter("topics.subscriptions.global_map", "/slam/mapping/map_global");
    node.declare_parameter("topics.subscriptions.visualization_map", "/slam/mapping/map_global");
    node.declare_parameter("topics.subscriptions.slam_mode", "/slam/slam_mode");

    // out topics
    node.declare_parameter("topics.out.point_cloud_available", "point_cloud_available");
    node.declare_parameter("topics.out.mapping_point_cloud_available",
                           "mapping_point_cloud_available");
    node.declare_parameter("topics.out.trajectories", "trajectories");

    // services (server)
    node.declare_parameter("services.server.add_floor", "add_floor");
    node.declare_parameter("services.server.delete_floor", "delete_floor");
    node.declare_parameter("services.server.clear_floor", "clear_floor");
    node.declare_parameter("services.server.get_floors", "get_floors");
    node.declare_parameter("services.server.select_mapping_floor", "select_mapping_floor");
    node.declare_parameter("services.server.get_mapping_floor", "get_mapping_floor");
    node.declare_parameter("services.server.select_view_floor", "select_view_floor");
    node.declare_parameter("services.server.get_view_floor", "get_view_floor");
    node.declare_parameter("services.server.set_floor_name", "set_floor_name");
    node.declare_parameter("services.server.export_map", "export_map");
    node.declare_parameter("services.server.clear_map", "clear_map");
    node.declare_parameter("services.server.get_software_versions", "get_software_versions");
    node.declare_parameter("services.server.clear_path", "clear_path");

    // services (client)
    node.declare_parameter("services.client.set_slam_mode", "/slam/set_slam_mode");

    // Read parameters into members
    version_file_path = node.get_parameter("version_file_path").as_string();
    map_path          = node.get_parameter("map_path").as_string();
    map_save_interval_ms =
        std::chrono::duration<int, std::milli>{node.get_parameter("map_save_interval").as_int()};
    slam_mode_timeout_ms =
        std::chrono::duration<int, std::milli>{node.get_parameter("slam_timeout").as_int()};
    topic_robot_status = node.get_parameter("topics.subscriptions.robot_status").as_string();
    topic_trajectory   = node.get_parameter("topics.subscriptions.trajectory").as_string();
    topic_global_map   = node.get_parameter("topics.subscriptions.global_map").as_string();
    topic_visualization_map =
        node.get_parameter("topics.subscriptions.visualization_map").as_string();

    topic_point_cloud_available =
        node.get_parameter("topics.out.point_cloud_available").as_string();
    topic_mapping_point_cloud_available =
        node.get_parameter("topics.out.mapping_point_cloud_available").as_string();
    topic_trajectories = node.get_parameter("topics.out.trajectories").as_string();
    topic_slam_mode    = node.get_parameter("topics.subscriptions.slam_mode").as_string();

    srv_add_floor    = node.get_parameter("services.server.add_floor").as_string();
    srv_delete_floor = node.get_parameter("services.server.delete_floor").as_string();
    srv_clear_floor  = node.get_parameter("services.server.clear_floor").as_string();
    srv_get_floors   = node.get_parameter("services.server.get_floors").as_string();
    srv_select_mapping_floor =
        node.get_parameter("services.server.select_mapping_floor").as_string();
    srv_get_mapping_floor = node.get_parameter("services.server.get_mapping_floor").as_string();
    srv_select_view_floor = node.get_parameter("services.server.select_view_floor").as_string();
    srv_get_view_floor    = node.get_parameter("services.server.get_view_floor").as_string();
    srv_set_floor_name    = node.get_parameter("services.server.set_floor_name").as_string();
    srv_export_map        = node.get_parameter("services.server.export_map").as_string();
    srv_clear_map         = node.get_parameter("services.server.clear_map").as_string();
    srv_get_software_versions =
        node.get_parameter("services.server.get_software_versions").as_string();
    srv_clear_path = node.get_parameter("services.server.clear_path").as_string();

    srv_set_slam_mode = node.get_parameter("services.client.set_slam_mode").as_string();
}
