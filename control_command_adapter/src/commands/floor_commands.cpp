// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include "commands/floor_commands.hpp"

// C++ Standard Library

// ROS2 Core
#include <rclcpp/rclcpp.hpp>

// Project headers
#include "service_helpers.hpp"

FloorCommands::FloorCommands(rclcpp::Node::SharedPtr node, const Configuration &config)
    : node_(node)
{
    cmd_timeout_ = config.cmd_timeout_;

    // Floor commands
    this->add_floor_client_ =
        this->node_->create_client<interface_msgs::srv::SetString>(config.add_floor_service_);
    this->delete_floor_client_ = this->node_->create_client<interface_msgs::srv::ActionOnIndex>(
        config.delete_floor_service_);
    this->clear_floor_client_ =
        this->node_->create_client<std_srvs::srv::Trigger>(config.clear_floor_service_);
    this->get_floors_client_ =
        this->node_->create_client<interface_msgs::srv::Listing>(config.get_floors_service_);
    this->get_mapping_floor_client_ = this->node_->create_client<interface_msgs::srv::GetIndex>(
        config.get_mapping_floor_service_);
    this->get_view_floor_client_ =
        this->node_->create_client<interface_msgs::srv::GetIndex>(config.get_view_floor_service_);
    this->select_view_floor_client_ =
        this->node_->create_client<interface_msgs::srv::ActionOnIndex>(
            config.select_view_floor_service_);
    this->select_mapping_floor_client_ =
        this->node_->create_client<interface_msgs::srv::ActionOnIndex>(
            config.select_mapping_floor_service_);
    this->set_floor_name_client_ = this->node_->create_client<interface_msgs::srv::UpdateStringAt>(
        config.set_floor_name_service_);
}

nlohmann::json FloorCommands::AddFloor(const nlohmann::json &params)
{
    std::string floor_name = params.at("FloorName");
    RCLCPP_INFO(node_->get_logger(), "Adding floor: %s.", floor_name.c_str());
    auto request  = std::make_shared<interface_msgs::srv::SetString::Request>();
    request->data = floor_name;
    return call_service_empty_response<interface_msgs::srv::SetString>(this->add_floor_client_,
                                                                       request, cmd_timeout_);
}

nlohmann::json FloorCommands::DeleteFloor(const nlohmann::json &params)
{
    int  floor_index = params.at("FloorIndex");
    auto request     = std::make_shared<interface_msgs::srv::ActionOnIndex::Request>();
    request->index   = floor_index;
    RCLCPP_INFO(node_->get_logger(), "Delete floor: %d.", floor_index);
    return call_service_empty_response<interface_msgs::srv::ActionOnIndex>(
        this->delete_floor_client_, request, cmd_timeout_);
}

nlohmann::json FloorCommands::ClearFloor()
{
    RCLCPP_INFO(node_->get_logger(), "Clear floor.");
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    return call_service_empty_response<std_srvs::srv::Trigger>(this->clear_floor_client_, request,
                                                               cmd_timeout_);
}

nlohmann::json FloorCommands::GetFloors()
{
    RCLCPP_INFO(node_->get_logger(), "Get floors.");
    auto request = std::make_shared<interface_msgs::srv::Listing::Request>();
    return call_service_with_list_response<interface_msgs::srv::Listing>(this->get_floors_client_,
                                                                         request, cmd_timeout_);
}

nlohmann::json FloorCommands::GetMappingFloor()
{
    RCLCPP_INFO(node_->get_logger(), "Get mapping floor.");
    auto request = std::make_shared<interface_msgs::srv::GetIndex::Request>();
    return call_service_with_int_response<interface_msgs::srv::GetIndex>(
        this->get_mapping_floor_client_, request, cmd_timeout_);
}

nlohmann::json FloorCommands::GetViewFloor()
{
    RCLCPP_INFO(node_->get_logger(), "Get view floor.");
    auto request = std::make_shared<interface_msgs::srv::GetIndex::Request>();
    return call_service_with_int_response<interface_msgs::srv::GetIndex>(
        this->get_view_floor_client_, request, cmd_timeout_);
}

nlohmann::json FloorCommands::SelectViewFloor(const nlohmann::json &params)
{
    int floor_index = params.at("FloorIndex");
    RCLCPP_INFO(node_->get_logger(), "Select view floor: %d.", floor_index);
    auto request   = std::make_shared<interface_msgs::srv::ActionOnIndex::Request>();
    request->index = floor_index;
    return call_service_empty_response<interface_msgs::srv::ActionOnIndex>(
        this->select_view_floor_client_, request, cmd_timeout_);
}

nlohmann::json FloorCommands::SelectMappingFloor(const nlohmann::json &params)
{
    int floor_index = params.at("FloorIndex");
    RCLCPP_INFO(node_->get_logger(), "Select mapping floor: %d.", floor_index);
    auto request   = std::make_shared<interface_msgs::srv::ActionOnIndex::Request>();
    request->index = floor_index;
    return call_service_empty_response<interface_msgs::srv::ActionOnIndex>(
        this->select_mapping_floor_client_, request, cmd_timeout_);
}

nlohmann::json FloorCommands::SetFloorName(const nlohmann::json &params)
{
    std::string floor_name  = params.at("FloorName");
    int         floor_index = params.at("FloorIndex");
    RCLCPP_INFO(node_->get_logger(), "Set floor name: %s %d.", floor_name.c_str(), floor_index);
    auto request   = std::make_shared<interface_msgs::srv::UpdateStringAt::Request>();
    request->index = floor_index;
    request->data  = floor_name;
    return call_service_empty_response<interface_msgs::srv::UpdateStringAt>(
        this->set_floor_name_client_, request, cmd_timeout_);
}
