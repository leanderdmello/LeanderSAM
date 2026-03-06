// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include "commands/map_commands.hpp"

// C++ Standard Library

// ROS2 Core
#include <rclcpp/rclcpp.hpp>

// Project headers
#include "service_helpers.hpp"

MapCommands::MapCommands(rclcpp::Node::SharedPtr node, const Configuration &config) : node_(node)
{
    cmd_timeout_ = config.cmd_timeout_;

    // Map commands
    this->export_map_client_ =
        this->node_->create_client<interface_msgs::srv::SetString>(config.export_map_service_);
    this->clear_map_client_ =
        this->node_->create_client<std_srvs::srv::Trigger>(config.clear_map_service_);
}

nlohmann::json MapCommands::ExportMap(const nlohmann::json &params)
{
    std::string label = params.at("Label");
    RCLCPP_INFO(node_->get_logger(), "Export map: %s.", label.c_str());
    auto request  = std::make_shared<interface_msgs::srv::SetString::Request>();
    request->data = label;
    return call_service_empty_response<interface_msgs::srv::SetString>(this->export_map_client_,
                                                                       request, cmd_timeout_);
}

nlohmann::json MapCommands::ClearMap()
{
    RCLCPP_INFO(node_->get_logger(), "Clear map.");
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    return call_service_empty_response<std_srvs::srv::Trigger>(this->clear_map_client_, request,
                                                               cmd_timeout_);
}
