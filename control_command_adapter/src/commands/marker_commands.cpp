// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include "commands/marker_commands.hpp"

// C++ Standard Library

// ROS2 Core
#include <rclcpp/rclcpp.hpp>

// Project headers
#include "service_helpers.hpp"

MarkerCommands::MarkerCommands(rclcpp::Node::SharedPtr node, const Configuration &config)
    : node_(node)
{
    cmd_timeout_ = config.cmd_timeout_;

    // Marker commands
    this->add_marker_client_ =
        this->node_->create_client<interface_msgs::srv::SetString>(config.add_marker_service_);
    this->remove_marker_client_ =
        this->node_->create_client<interface_msgs::srv::ActionOnOptionalIndex>(
            config.remove_marker_service_);
    this->get_markers_client_ =
        this->node_->create_client<interface_msgs::srv::Listing>(config.get_markers_service_);
    this->set_marker_label_client_ = this->node_->create_client<interface_msgs::srv::SetString>(
        config.set_marker_label_service_);
    this->set_marker_label_at_client_ =
        this->node_->create_client<interface_msgs::srv::UpdateStringAt>(
            config.set_marker_label_at_position_service_);
}

nlohmann::json MarkerCommands::AddMarker(const nlohmann::json &params)
{
    std::string marker_name = params.at("MarkerName");
    RCLCPP_INFO(node_->get_logger(), "Adding marker: %s.", marker_name.c_str());
    auto request  = std::make_shared<interface_msgs::srv::SetString::Request>();
    request->data = marker_name;
    return call_service_empty_response<interface_msgs::srv::SetString>(this->add_marker_client_,
                                                                       request, cmd_timeout_);
}

nlohmann::json MarkerCommands::RemoveMarker(const std::optional<nlohmann::json> &params)
{
    std::optional<int> marker_index = std::nullopt;
    auto request = std::make_shared<interface_msgs::srv::ActionOnOptionalIndex::Request>();
    if(params.has_value() && params.value().contains("MarkerIndex"))
    {
        marker_index   = params.value().at("MarkerIndex");
        request->index = marker_index.value();
        RCLCPP_INFO(node_->get_logger(), "Remove marker: %d.", marker_index.value());
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(), "Remove marker.");
    }
    return call_service_empty_response<interface_msgs::srv::ActionOnOptionalIndex>(
        this->remove_marker_client_, request, cmd_timeout_);
}

nlohmann::json MarkerCommands::GetMarkers()
{
    RCLCPP_INFO(node_->get_logger(), "Get markers.");
    auto request = std::make_shared<interface_msgs::srv::Listing::Request>();
    return call_service_with_list_response<interface_msgs::srv::Listing>(this->get_markers_client_,
                                                                         request, cmd_timeout_);
}

nlohmann::json MarkerCommands::SetMarkerLabel(const nlohmann::json &params)
{
    std::string        marker_label = params.at("MarkerLabel");
    std::optional<int> marker_index = std::nullopt;
    if(params.contains("MarkerIndex"))
    {
        marker_index = params.at("MarkerIndex");
        RCLCPP_INFO(node_->get_logger(), "Set marker label: %s %d.", marker_label.c_str(),
                    marker_index.value());
        auto request   = std::make_shared<interface_msgs::srv::UpdateStringAt::Request>();
        request->index = marker_index.value();
        request->data  = marker_label;
        return call_service_empty_response<interface_msgs::srv::UpdateStringAt>(
            this->set_marker_label_at_client_, request, cmd_timeout_);
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(), "Set marker label: %s.", marker_label.c_str());
        auto request  = std::make_shared<interface_msgs::srv::SetString::Request>();
        request->data = marker_label;
        return call_service_empty_response<interface_msgs::srv::SetString>(
            this->set_marker_label_client_, request, cmd_timeout_);
    }
}
