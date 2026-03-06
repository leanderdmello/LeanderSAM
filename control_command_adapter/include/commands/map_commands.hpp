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
#include "std_srvs/srv/trigger.hpp"

#include "interface_msgs/srv/set_string.hpp"

#include "configuration.hpp"

class MapCommands
{
public:
    MapCommands(rclcpp::Node::SharedPtr node, const Configuration &config);

    // JSON-RPC command
    // Map commands
    nlohmann::json ExportMap(const nlohmann::json &params);
    nlohmann::json ClearMap();

private:
    // ROS service clients
    // Map commands
    rclcpp::Client<interface_msgs::srv::SetString>::SharedPtr export_map_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr         clear_map_client_;

    rclcpp::Node::SharedPtr                node_;
    std::chrono::duration<int, std::milli> cmd_timeout_;
};
