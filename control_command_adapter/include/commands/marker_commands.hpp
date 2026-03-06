// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#pragma once

#include <memory>
#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"

#include "interface_msgs/srv/set_string.hpp"
#include "interface_msgs/srv/action_on_optional_index.hpp"
#include "interface_msgs/srv/listing.hpp"
#include "interface_msgs/srv/update_string_at.hpp"

#include "configuration.hpp"

class MarkerCommands
{
public:
    MarkerCommands(rclcpp::Node::SharedPtr node, const Configuration &config);

    // JSON-RPC command handlersFloorCommands
    // Marker commands
    nlohmann::json AddMarker(const nlohmann::json &params);
    nlohmann::json RemoveMarker(const std::optional<nlohmann::json> &params);
    nlohmann::json GetMarkers();
    nlohmann::json SetMarkerLabel(const nlohmann::json &params);

private:
    // ROS service clients
    // Marker commands
    rclcpp::Client<interface_msgs::srv::SetString>::SharedPtr             add_marker_client_;
    rclcpp::Client<interface_msgs::srv::ActionOnOptionalIndex>::SharedPtr remove_marker_client_;
    rclcpp::Client<interface_msgs::srv::Listing>::SharedPtr               get_markers_client_;
    rclcpp::Client<interface_msgs::srv::SetString>::SharedPtr             set_marker_label_client_;
    rclcpp::Client<interface_msgs::srv::UpdateStringAt>::SharedPtr set_marker_label_at_client_;

    rclcpp::Node::SharedPtr                node_;
    std::chrono::duration<int, std::milli> cmd_timeout_;
};
